import os

base_dir = '/home/david/Documents/Research/ASSIST-U/output/cao_test/'
base_list = sorted(os.listdir(base_dir))

for video_name in base_list:
    depth_folder = os.path.join(base_dir, video_name, 'depth')
    raw_folder = os.path.join(base_dir, video_name, 'raw')
    render_folder = os.path.join(base_dir, video_name, 'render')

    if not all(os.path.isdir(f) for f in [depth_folder, raw_folder, render_folder]):
        print(f"Skipping {video_name} due to missing folder.")
        continue

    # Helper function to get base names
    def extract_base_names(folder, suffix):
        return {
            f.split('_')[0]
            for f in os.listdir(folder)
            if f.endswith(suffix)
        }

    depth_bases = extract_base_names(depth_folder, '_dmap.png')
    raw_bases = extract_base_names(raw_folder, '_dmap.exr')
    render_bases = extract_base_names(render_folder, '_render.png')

    common_bases = depth_bases & raw_bases & render_bases

    # Delete non-matching files
    for folder, suffix in [
        (depth_folder, '_dmap.png'),
        (raw_folder, '_dmap.exr'),
        (render_folder, '_render.png')
    ]:
        for f in os.listdir(folder):
            base = f.split('_')[0]
            if base not in common_bases and f.endswith(suffix):
                f_path = os.path.join(folder, f)
                os.remove(f_path)
                print(f"Deleted: {f_path}")
