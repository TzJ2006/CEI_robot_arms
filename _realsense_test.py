"""Save one color frame from the Astra Pro camera used by the policy loop."""
from pathlib import Path

import cv2

from policy_link import capture_color_bgr

out = Path(__file__).with_name("_camera_test")
out.mkdir(exist_ok=True)
frame = capture_color_bgr()
color_path = out / "astra_color.png"
cv2.imwrite(str(color_path), frame)
print(
    f"color_shape={frame.shape} color_mean={float(frame.mean()):.1f} "
    f"path={color_path}"
)
