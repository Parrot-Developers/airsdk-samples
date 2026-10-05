#!/usr/bin/env python3
"""Export ssd300_vgg16 (COCO-pretrained, used as-is, no training) to an ONNX
graph matching what services/object_detector/processing.cpp expects:

  - raw 0-255 RGB input, NHWC [1, 300, 300, 3] (mean-subtraction is baked
    into the graph; the mission does no normalization on its side);
  - two outputs, "bbox_regression" [1, 8732, 4, 1] and "cls_logits"
    [1, 8732, 91, 1], read back by name in onInferResult().

Also dumps the anchor generator's scales/steps/aspect_ratios to
anchor_config.json: diff it against generate_ssd_priors() in processing.cpp,
which must use the exact same anchor geometry for decoded boxes to be
correct.
"""
import argparse
import glob
import json
import os

import numpy as np
import torch
import torch.nn as nn
from PIL import Image
from torchvision.models.detection import SSD300_VGG16_Weights, ssd300_vgg16

IMAGE_SIZE = 300


class FlatScoringHead(nn.Module):
    """Same computation as torchvision's SSDScoringHead.forward(), but stays
    at rank 4 throughout (every SNPE backend caps tensor rank at 4): permutes
    the raw (N, A*K, H, W) conv output straight to (N, H, W, A*K) and
    reshapes directly to (N, H*W*A, K), instead of going through a rank-5
    (N, A, K, H, W) intermediate shape."""

    def __init__(self, scoring_head):
        super().__init__()
        self.module_list = scoring_head.module_list
        self.num_columns = scoring_head.num_columns

    def forward(self, x):
        all_results = []
        for features, module in zip(x, self.module_list):
            results = module(features)
            n, _, h, w = results.shape
            results = results.permute(0, 2, 3, 1)  # (N, H, W, A*K)
            results = results.reshape(n, -1, self.num_columns)  # (N, H*W*A, K)
            all_results.append(results)
        return torch.cat(all_results, dim=1)


class ExportableSSD(nn.Module):
    """Backbone + head only: no torchvision transform (resize/batching) and
    no postprocessing (decode/NMS) -- processing.cpp already does both of
    those itself, against its own generate_ssd_priors()."""

    def __init__(self, ssd):
        super().__init__()
        self.backbone = ssd.backbone
        self.head_regression = FlatScoringHead(ssd.head.regression_head)
        self.head_classification = FlatScoringHead(ssd.head.classification_head)
        # ssd.transform.image_mean is calibrated for a [0,1]-scaled image
        # (image_std = 1/255, i.e. no extra scaling beyond mean-subtraction).
        # Combined: normalize(x) = (x/255 - mean) / (1/255) = x - mean*255.
        mean_0_255 = torch.tensor(ssd.transform.image_mean).view(1, 3, 1, 1) * 255.0
        self.register_buffer("mean_0_255", mean_0_255)

    def forward(self, x_nchw):
        # NCHW in: snpe-onnx-to-dlc assumes ONNX image inputs are NCHW and
        # transposes them to its own NHWC runtime layout itself -- declaring
        # (and permuting from) NHWC here instead makes it misread which axis
        # is which and insert a wrong transpose.
        x = x_nchw - self.mean_0_255
        features = list(self.backbone(x).values())
        bbox_regression = self.head_regression(features).unsqueeze(-1)  # (1, 8732, 4, 1)
        cls_logits = self.head_classification(features).unsqueeze(-1)  # (1, 8732, 91, 1)
        return bbox_regression, cls_logits


def dump_anchor_config(ssd, out_path):
    gen = ssd.anchor_generator
    config = {
        "aspect_ratios": gen.aspect_ratios,
        "scales": gen.scales,
        "steps": gen.steps,
        "clip": gen.clip,
    }
    with open(out_path, "w") as f:
        json.dump(config, f, indent=2)
    print(f"wrote {out_path}")
    print(
        "*** diff this against generate_ssd_priors() in processing.cpp "
        "before trusting the exported network -- scales/steps/clip must "
        "match exactly, or decoded boxes will be wrong ***"
    )


def export_onnx(ssd, out_path):
    wrapper = ExportableSSD(ssd).eval()
    dummy_input = torch.zeros(1, 3, IMAGE_SIZE, IMAGE_SIZE, dtype=torch.float32)
    torch.onnx.export(
        wrapper,
        dummy_input,
        out_path,
        input_names=["input"],
        output_names=["bbox_regression", "cls_logits"],
        opset_version=11,
        dynamo=False,  # keep the legacy TorchScript-tracing exporter: its
        # module-scope-based op naming is what settings.outputLayers
        # (processing.cpp) assumes; the newer dynamo exporter names nodes
        # differently and isn't guaranteed to match.
    )
    print(f"wrote {out_path}")


def write_calibration_inputs(calib_images_dir, out_dir):
    """Raw float32 NHWC 0-255 RGB, resized to 300x300 -- exactly what
    preProcessFrame() produces before calling infer(); normalization is
    baked into the exported graph, see ExportableSSD."""
    raw_dir = os.path.join(out_dir, "calib_raw")
    os.makedirs(raw_dir, exist_ok=True)
    input_list_path = os.path.join(out_dir, "input_list.txt")

    image_paths = sorted(
        p
        for ext in ("*.jpg", "*.jpeg", "*.png")
        for p in glob.glob(os.path.join(calib_images_dir, ext))
    )
    if not image_paths:
        raise SystemExit(f"no calibration images found in {calib_images_dir}")

    with open(input_list_path, "w") as input_list:
        for path in image_paths:
            img = Image.open(path).convert("RGB").resize((IMAGE_SIZE, IMAGE_SIZE))
            arr = np.asarray(img, dtype=np.float32)  # HWC, 0-255, no scaling
            raw_path = os.path.join(
                raw_dir, os.path.splitext(os.path.basename(path))[0] + ".raw"
            )
            arr.tofile(raw_path)
            input_list.write(raw_path + "\n")

    print(f"wrote {len(image_paths)} calibration inputs, list at {input_list_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib-images-dir", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ssd = ssd300_vgg16(weights=SSD300_VGG16_Weights.COCO_V1)
    ssd.eval()

    dump_anchor_config(ssd, os.path.join(args.out, "anchor_config.json"))
    export_onnx(ssd, os.path.join(args.out, "object_detector.onnx"))
    write_calibration_inputs(args.calib_images_dir, args.out)


if __name__ == "__main__":
    main()
