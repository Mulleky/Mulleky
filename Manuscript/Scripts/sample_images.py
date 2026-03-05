import os
import shutil

# Paths
source_dir = "/home/carlos/Desktop/Manuscript/Table Scene/GroundTruth/Images"
output_base = "/home/carlos/Desktop/Manuscript/Table Scene/Colmap"

# Protected images that must appear in every dataset
protected = ["IMG_0801.JPG", "IMG_0860.JPG"]

# Target image counts
tiers = [10, 20, 35, 50, 65]

# Get all images sorted by filename (preserves shooting order)
all_images = sorted([f for f in os.listdir(source_dir)
                     if f.lower().endswith(('.jpg', '.jpeg', '.png', '.tiff', '.tif'))])

# Split into two rings based on shooting pattern
# Ring 1: above POV (IMG_0801 - IMG_0836)
# Ring 2: lower POV (IMG_0837 - IMG_0882)
ring1 = [f for f in all_images if 801 <= int(f.replace("IMG_", "").replace(".JPG", "").replace(".jpg", "")) <= 836]
ring2 = [f for f in all_images if 837 <= int(f.replace("IMG_", "").replace(".JPG", "").replace(".jpg", "")) <= 882]

print(f"Ring 1 (above POV): {len(ring1)} images")
print(f"Ring 2 (lower POV): {len(ring2)} images")
print(f"Total: {len(ring1) + len(ring2)} images\n")

# Proportion of each ring in the full dataset
total = len(ring1) + len(ring2)
ratio1 = len(ring1) / total
ratio2 = len(ring2) / total


def interval_sample(image_list, n, protected_in_group):
    """
    Sample n images from image_list using even intervals.
    Protected images already in this group are locked in and
    remaining slots are filled with interval-sampled images.
    """
    # Remove protected images from the pool to avoid double-counting
    pool = [f for f in image_list if f not in protected_in_group]
    locked = [f for f in protected_in_group if f in image_list]
    
    n_needed = n - len(locked)
    
    if n_needed <= 0:
        return locked
    
    if n_needed >= len(pool):
        return locked + pool
    
    # Interval sampling from pool
    step = len(pool) / n_needed
    sampled = [pool[int(i * step)] for i in range(n_needed)]
    
    return locked + sampled


for count in tiers:
    # Allocate images proportionally between rings
    # Reserve 2 slots for protected images (one per ring)
    n1_protected = 1 if "IMG_0801.JPG" in ring1 else 0  # IMG_0801 is in ring1
    n2_protected = 1 if "IMG_0860.JPG" in ring2 else 0  # IMG_0860 is in ring2

    # Proportional split of remaining slots
    remaining = count - len(protected)
    n1 = round(remaining * ratio1) + n1_protected
    n2 = (count - n1)

    # Clamp to available images
    n1 = min(n1, len(ring1))
    n2 = min(n2, len(ring2))

    sampled1 = interval_sample(ring1, n1, ["IMG_0801.JPG"])
    sampled2 = interval_sample(ring2, n2, ["IMG_0860.JPG"])

    final_set = list(set(sampled1 + sampled2))

    # Create output folder
    folder_name = f"{count}_images"
    folder_path = os.path.join(output_base, folder_name)
    os.makedirs(folder_path, exist_ok=True)

    # Copy images
    for img in final_set:
        src = os.path.join(source_dir, img)
        dst = os.path.join(folder_path, img)
        shutil.copy2(src, dst)

    print(f"Created {folder_name}: {len(sampled1)} from ring1, {len(sampled2)} from ring2 — total {len(final_set)}")

print("\nDone.")