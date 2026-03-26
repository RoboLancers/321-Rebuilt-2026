"""
Processes robot dashboard images:
  1. Removes white/near-white backgrounds (makes them transparent).
  2. Pads side-arm.png to the exact same canvas size as side-view.png
     so the arm overlay aligns pixel-perfectly when both use object-fit:contain.
  3. Also removes the background from front-view.png.

Run from the robot-dashboard directory:
    python scripts/process_images.py
"""

from pathlib import Path
import numpy as np
from PIL import Image


IMAGES_DIR = Path(__file__).parent.parent / "public" / "images"
WHITE_THRESHOLD = 230  # pixels with R,G,B all above this are treated as background


def remove_white_bg(img: Image.Image, threshold: int = WHITE_THRESHOLD) -> Image.Image:
    """Replace near-white pixels with full transparency."""
    rgba = img.convert("RGBA")
    data = np.array(rgba, dtype=np.uint8)

    r, g, b = data[:, :, 0], data[:, :, 1], data[:, :, 2]

    # Mark pixels as background if all channels are bright.
    is_white = (r > threshold) & (g > threshold) & (b > threshold)

    # Also catch off-white caused by JPEG/PNG anti-aliasing on robot edges:
    # if a pixel is majority white (average channel > threshold * 0.9) it goes transparent.
    avg = r.astype(np.int32) + g.astype(np.int32) + b.astype(np.int32)
    is_near_white = avg > threshold * 2.7  # ~= all three > threshold * 0.9

    data[:, :, 3][is_white | is_near_white] = 0

    return Image.fromarray(data, "RGBA")


def pad_to_size(img: Image.Image, target_w: int, target_h: int) -> Image.Image:
    """
    Centre img on a transparent canvas of (target_w, target_h).
    Used to make the arm image the same dimensions as the robot body image.
    """
    canvas = Image.new("RGBA", (target_w, target_h), (0, 0, 0, 0))
    x = (target_w - img.width) // 2
    y = (target_h - img.height) // 2
    canvas.paste(img, (x, y), mask=img if img.mode == "RGBA" else None)
    return canvas


def process():
    side_view_path = IMAGES_DIR / "side-view.png"
    side_arm_path  = IMAGES_DIR / "side-arm.png"
    front_view_path = IMAGES_DIR / "front-view.png"

    # ── Side view (robot body) ──────────────────────────────────────────────
    if not side_view_path.exists():
        print(f"  [SKIP] {side_view_path.name} not found")
    else:
        img = Image.open(side_view_path)
        clean = remove_white_bg(img)
        clean.save(side_view_path)
        print(f"  [OK] {side_view_path.name}  {img.size} → transparent bg")
        robot_w, robot_h = clean.size

        # ── Side arm ───────────────────────────────────────────────────────
        if not side_arm_path.exists():
            print(f"  [SKIP] {side_arm_path.name} not found")
        else:
            arm = Image.open(side_arm_path)
            arm_clean = remove_white_bg(arm)

            if arm_clean.size != (robot_w, robot_h):
                print(
                    f"  [PAD]  {side_arm_path.name}  "
                    f"{arm_clean.size} → padded to {(robot_w, robot_h)}"
                )
                arm_clean = pad_to_size(arm_clean, robot_w, robot_h)
            else:
                print(f"  [OK]  {side_arm_path.name}  sizes already match")

            arm_clean.save(side_arm_path)
            print(f"  [OK] {side_arm_path.name}  saved")

    # ── Front view ─────────────────────────────────────────────────────────
    if not front_view_path.exists():
        print(f"  [SKIP] {front_view_path.name} not found")
    else:
        img = Image.open(front_view_path)
        clean = remove_white_bg(img)
        clean.save(front_view_path)
        print(f"  [OK] {front_view_path.name}  {img.size} → transparent bg")


if __name__ == "__main__":
    print(f"Images directory: {IMAGES_DIR.resolve()}\n")
    process()
    print("\nDone. Reload the browser to see the changes.")
