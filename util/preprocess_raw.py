import os
import numpy as np
import imageio.v3 as iio

base_dir = '/home/david/Documents/Research/ASSIST-U/output/cao_test/'
base_list = sorted(os.listdir(base_dir))

for video_name in base_list:
    raw_folder = os.path.join(base_dir, video_name, 'raw')
    output_folder = os.path.join(base_dir, video_name, 'preprocessed_raw')

    if not os.path.isdir(raw_folder):
        print(f"Skipping {video_name}: raw folder missing.")
        continue

    os.makedirs(output_folder, exist_ok=True)

    for filename in os.listdir(raw_folder):
        if filename.endswith('_dmap.exr'):
            raw_path = os.path.join(raw_folder, filename)
            output_path = os.path.join(output_folder, filename.replace('.exr', '.npy'))

            try:
                # Load EXR using imageio (expects OpenEXR plugin)
                depth_map = iio.imread(raw_path).astype(np.float32)  # shape: (H, W)

                # Convert from [0, 1] → [0.1, 100] mm
                converted = (depth_map * (100.0 - 0.1) + 0.1).astype(np.float16)
                np.savez_compressed(output_path.replace('.npy', '.npz'), depth=converted)

                print(f"Saved: {output_path}")

            except Exception as e:
                print(f"Failed to process {raw_path}: {e}")
