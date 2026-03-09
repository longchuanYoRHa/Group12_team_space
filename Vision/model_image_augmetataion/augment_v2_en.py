import cv2
import numpy as np
import os

# ================= Configuration =================
# Original image folder
INPUT_DIR = 'images' 
# Augmented image save location
OUTPUT_DIR = 'augmented_dataset_v2'
# ===========================================================

def create_output_dir():
    """Create output folder"""
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

def save_image(img, filename, suffix):
    """Save image to output folder, automatically add suffix"""
    name, ext = os.path.splitext(filename)
    new_filename = f"{name}_{suffix}{ext}"
    save_path = os.path.join(OUTPUT_DIR, new_filename)
    cv2.imwrite(save_path, img)

# --- Image Processing Functions ---

def adjust_brightness(img, factor):
    """
    Adjust brightness
    factor > 1: Overexposure
    factor < 1: Underexposure
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    # Use cv2.multiply to prevent overflow
    v = cv2.multiply(v, factor)
    # Limit values between 0-255
    v = np.clip(v, 0, 255).astype(hsv.dtype)
    
    hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def add_noise(img):
    """Add Gaussian noise (Gaussian Noise)"""
    row, col, ch = img.shape
    mean = 0
    var = 50 # Noise variance, the greater the noise, the more points
    sigma = var ** 0.5
    gauss = np.random.normal(mean, sigma, (row, col, ch))
    gauss = gauss.reshape(row, col, ch)
    noisy = img + gauss
    return np.clip(noisy, 0, 255).astype(np.uint8)

def rotate_image(img, angle_code):
    """
    Rotate image
    angle_code can be:
    cv2.ROTATE_90_CLOCKWISE (90 degrees clockwise)
    cv2.ROTATE_180 (180 degrees)
    """
    return cv2.rotate(img, angle_code)

def flip_image(img, flip_code):
    """
    Flip image
    1: Horizontal flip (Horizontal)
    0: Vertical flip (Vertical)
    """
    return cv2.flip(img, flip_code)

# --- Main Execution ---

def main():
    create_output_dir()
    
    # Supported image formats
    supported_formats = ('.jpg', '.jpeg', '.png', '.bmp')
    # Read all images with supported formats
    files = [f for f in os.listdir(INPUT_DIR) if f.lower().endswith(supported_formats)]
    
    print(f"🚀 Found {len(files)} original images. Starting augmentation (7x expansion)...")
    
    count = 0
    for file in files:
        img_path = os.path.join(INPUT_DIR, file)
        img = cv2.imread(img_path)
        
        if img is None:
            print(f" Warning: Could not read image {file}, skipping.")
            continue

        # === 1. Save original image (Original) ===
        save_image(img, file, "original")

        # === Geometric Transformations (Geometric Transformations) ===
        
        # 2. Horizontal flip (Horizontal Flip) - simulate mirror
        img_flip_h = flip_image(img, 1)
        save_image(img_flip_h, file, "flip_h")

        # 3. Vertical flip (Vertical Flip) - simulate inversion
        img_flip_v = flip_image(img, 0)
        save_image(img_flip_v, file, "flip_v")

        # 4. Rotate 90 degrees (Rotate 90 deg clockwise)
        img_rot90 = rotate_image(img, cv2.ROTATE_90_CLOCKWISE)
        save_image(img_rot90, file, "rot90")

        # 5. Rotate 180 degrees (Rotate 180 deg)
        img_rot180 = rotate_image(img, cv2.ROTATE_180)
        save_image(img_rot180, file, "rot180")

        # === Pixel-level Transformations (Pixel-level Transformations) ===
        
        # 6. Overexposure (Overexposure) - brightness increase 60%
        img_bright = adjust_brightness(img, 1.6)
        save_image(img_bright, file, "bright")

        # 7. Underexposure (Underexposure) - brightness decrease 40%
        img_dark = adjust_brightness(img, 0.6)
        save_image(img_dark, file, "dark")

        # 8. Mixed effects: rotate 90 degrees + noise (Rotate 90 + Noise)
        # Rotate first, then add noise, simulate a more恶劣的识别环境
        img_mix = add_noise(img_rot90)
        save_image(img_mix, file, "rot90_noise")

        count += 1
        # Print progress every 50 images processed
        if count % 50 == 0:
            print(f"Processed {count} images (Generated {count*8} files so far)...")

    print("="*50)
    print(f"Processing Completed Successfully!")
    print(f"Original Images:  {len(files)}")
    print(f"Total Augmented:  {len(files) * 8}")
    print(f"Output Directory: {OUTPUT_DIR}")
    print("="*50)
    print(f" NEXT STEP: Open '{OUTPUT_DIR}' in your labeling tool (e.g., Roboflow/LabelImg) and start labeling.")

if __name__ == "__main__":
    main()