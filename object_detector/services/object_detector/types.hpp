/**
 * Copyright (c) 2026 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Parrot Company nor the names
 *   of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written
 *   permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#pragma once

#include <algorithm>

// Bounding box in normalized coordinates.
struct Box {
	float x1, y1, x2, y2;
};

// A single detection: class, score and box.
struct Detection {
	int class_id;
	float score;
	Box box;
};

// IoU between two boxes, used by Processing's per-class NMS.
inline float computeIou(const Box &a, const Box &b)
{
	float x1 = std::max(a.x1, b.x1);
	float y1 = std::max(a.y1, b.y1);
	float x2 = std::min(a.x2, b.x2);
	float y2 = std::min(a.y2, b.y2);

	float w = std::max(0.f, x2 - x1);
	float h = std::max(0.f, y2 - y1);
	float inter = w * h;

	float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
	float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
	float union_ = area_a + area_b - inter;

	return inter / (union_ + 1e-6f);
}
