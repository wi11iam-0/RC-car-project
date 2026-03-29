# image_tester.py — batch test the policy on all images in a folder

import os
import cv2
from policy_model_YOLO import make_policy, Action

# =========================
# settings
# 
TEST_IMAGES_DIR      = r"C:\Users\User\Documents\VS Code\Python\ESP32-CAM communication\CV models\test_images"
ANNOTATED_IMAGES_DIR = r"C:\Users\User\Documents\VS Code\Python\ESP32-CAM communication\CV models\annotated_test_images"

POLICY_KIND   = "yolo"           # "heuristic" or "yolo"
FPS_LIMIT     = 10.0             # high fps, but we also turn the gate off for batch testing
SHOW_PREVIEW  = False            # show each annotated image
# =========================


def put_hud(frame, text: str):
    """
    draw a simple bar at the top with text on it

    font size and thickness scale with the image size so the label stays readable
    on both small and large images
    """
    out = frame.copy()
    h, w = out.shape[:2]

    # make the top bar scale with image height, but do not let it get too tiny
    bar_height = max(int(0.08 * h), 24)

    # scale text size roughly from image height, with some limits so it stays sensible
    font_scale = max(0.5, min(2.0, h / 600.0))

    # make line thickness grow a bit for larger images
    thickness = max(1, int(h / 400))

    # draw the black bar across the top
    cv2.rectangle(out, (0, 0), (w, bar_height), (0, 0, 0), -1)

    # place the text with a bit of padding so it sits nicely in the bar
    baseline_y = int(bar_height * 0.7)
    cv2.putText(
        out,
        text,
        (10, baseline_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )

    return out


def iter_image_paths(folder: str):
    """yield full paths for image files in the folder"""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
    for name in os.listdir(folder):
        _, ext = os.path.splitext(name)
        if ext.lower() in exts:
            yield os.path.join(folder, name)


def main():
    if not os.path.isdir(TEST_IMAGES_DIR):
        raise FileNotFoundError(f"Test images folder does not exist: {TEST_IMAGES_DIR}")

    # make sure the output folder exists before trying to save anything
    os.makedirs(ANNOTATED_IMAGES_DIR, exist_ok=True)
    print("[INFO] Output folder:", os.path.abspath(ANNOTATED_IMAGES_DIR))

    # build the policy we want to test
    policy = make_policy(POLICY_KIND, fps=FPS_LIMIT)

    # turn the fps gate off for batch image testing
    # that way predict() runs straight away on every image
    if hasattr(policy, "gate"):
        try:
            policy.gate.min_interval_s = 0.0
            policy.gate._last_time = 0.0
        except Exception:
            pass

    print(f"[INFO] Reading images from: {os.path.abspath(TEST_IMAGES_DIR)}")
    print(f"[INFO] Policy: {POLICY_KIND}")

    processed = 0

    for img_path in iter_image_paths(TEST_IMAGES_DIR):
        print("\n[INFO] Processing:", img_path)
        img = cv2.imread(img_path)
        if img is None:
            print(f"[WARN] Could not read image: {img_path} — skipping.")
            continue

        # run the policy on this image
        action = policy.predict(img)
        if action is None:
            if hasattr(policy, "gate"):
                policy.gate._last_time = -1.0
            action = policy.predict(img)

        print(f"[RESULT] {os.path.basename(img_path)} -> {action}")

        label = f"ACTION: {action}   |   policy={POLICY_KIND}"
        vis = put_hud(img, label)

        # save the annotated version using the same filename in the output folder
        out_path = os.path.join(ANNOTATED_IMAGES_DIR, os.path.basename(img_path))
        print("[DEBUG] Attempting to save to:", out_path)
        ok = cv2.imwrite(out_path, vis)
        print("[DEBUG] cv2.imwrite return value:", ok)

        if not ok:
            print(f"[WARN] Failed to save annotated image: {out_path}")
        else:
            print(f"[INFO] Saved annotated image -> {out_path}")
            processed += 1

        if SHOW_PREVIEW:
            cv2.imshow("RC Image Test", vis)
            key = cv2.waitKey(0) & 0xFF

            # press ESC if you want to stop early
            if key == 27:
                break

    if SHOW_PREVIEW:
        cv2.destroyAllWindows()

    print(f"\n[INFO] Finished. Successfully processed {processed} images.")


if __name__ == "__main__":
    main()