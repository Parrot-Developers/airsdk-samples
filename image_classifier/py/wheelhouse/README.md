# wheelhouse

Drop drone-compatible Python wheels (`.whl`) here.

`numpy` ships compiled extensions, and the generic `manylinux_*_aarch64` wheel
pip fetches is not compatible with the drone's libc (import fails with an
`undefined symbol` / GLIBC error). Provide a `numpy` wheel built for the drone
target in this directory; the AirSDK build passes it to pip via `-f wheelhouse`
(pip prefers it over the index).

The mission's only native Python dependency is `numpy` (preprocessing and
post-processing are numpy-only, no OpenCV).
