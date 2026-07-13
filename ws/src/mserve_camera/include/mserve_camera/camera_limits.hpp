#pragma once

namespace mserve_camera {

constexpr int kWidthMin  = 160;
constexpr int kWidthMax  = 1920;
constexpr int kHeightMin = 120;
constexpr int kHeightMax = 1080;

// Requested via VIDIOC_S_PARM after format negotiation — V4l2CameraDevice
// doesn't wrap this ioctl, and without it the driver keeps whatever frame
// interval was last active (observed ~12.6Hz instead of the format table's
// 30Hz at 640x480 YUYV). Not resolution-aware: if a resolution/format combo
// that only supports a lower rate is requested, the driver just clamps to
// its actual max — this is a request, not a guarantee.
//
// KNOWN NOT TO WORK (2026-07-13): verified against real hardware that the
// second-fd approach in request_frame_rate() does not actually change the
// streamed rate (still ~12.6Hz measured after this "succeeds"). The
// assumption that frame interval is UVC device-level, not per-fd, was wrong
// for this driver — VIDIOC_S_PARM on a second fd is silently a no-op for the
// fd that actually calls STREAMON. See docs/todo.md. Left in place because
// it's harmless (still just a WARN on failure), not because it's believed
// to work.
constexpr int kTargetFps = 30;

// JPEG quality for camera/image_raw/compressed (cv::IMWRITE_JPEG_QUALITY).
constexpr int kJpegQualityDefault = 80;
constexpr int kJpegQualityMin = 1;
constexpr int kJpegQualityMax = 100;

}  // namespace mserve_camera
