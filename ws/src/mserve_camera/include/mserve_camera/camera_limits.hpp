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
// KNOWN NOT TO WORK (verified on real hardware): the second-fd approach in
// request_frame_rate() does not change the streamed rate (~12.6Hz measured
// after it "succeeds") — frame interval is per-fd state for this driver, so
// VIDIOC_S_PARM on a second fd is silently a no-op for the fd that actually
// calls STREAMON. See docs/camera/todo.md. Left in place because it's a
// harmless WARN on failure, not because it's believed to work.
constexpr int kTargetFps = 30;

// JPEG quality for camera/image_raw/compressed (cv::IMWRITE_JPEG_QUALITY).
constexpr int kJpegQualityDefault = 80;
constexpr int kJpegQualityMin = 1;
constexpr int kJpegQualityMax = 100;

// Ceiling on capture_loop()'s own iteration rate — see that function's
// comment for why. Default (camera_node.hpp) sits above kTargetFps's
// observed real ceiling (~12.6Hz) so it costs nothing normally; max is
// deliberately well below any real webcam frame rate, since this exists to
// cap runaway behavior, not to let someone configure it away.
constexpr double kCaptureRateHzMin = 1.0;
constexpr double kCaptureRateHzMax = 30.0;

}  // namespace mserve_camera
