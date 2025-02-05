import os
import cv2
from tqdm import tqdm

def dir2vid(savedir, name):
    # Path to the folder containing the images
    image_folder = f'{savedir}/{name}/render/'
    depth_folder = f'{savedir}/{name}/depth/'
    phantom_folder = f'../output/phantom/'

    # Output video file name
    output_video = f'{savedir}/{name}.mp4'

    # Frame rate (frames per second) for the output video
    frame_rate = 30  # You can adjust this as needed

    # Get the list of image files in the folder and sort them alphabetically
    image_files = sorted([os.path.join(image_folder, file) for file in os.listdir(image_folder) if file.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))])
    depth_files = sorted([os.path.join(depth_folder, file) for file in os.listdir(depth_folder) if file.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))])
    phantom_files = sorted([os.path.join(phantom_folder, file[:-11] + file[-4:]) for file in os.listdir(image_folder) if file.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))])

    if not image_files:
        print("No image files found in the folder.")
        exit()


    # Open the first image to get its dimensions
    first_image = cv2.imread(image_files[0])
    height, width, layers = first_image.shape

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can change the codec as needed
    video = cv2.VideoWriter(output_video, fourcc, frame_rate, (width * 3, height))
    pbar = tqdm(total=len(image_files))
    for image_file, depth_file, phantom_file in zip(image_files, depth_files, phantom_files):
        image = cv2.imread(image_file)
        depth = cv2.imread(depth_file)
        # for RtoP, flip by 180
        # image = cv2.rotate(image, cv2.ROTATE_180)
        # depth = cv2.rotate(depth, cv2.ROTATE_180)
        # image = cv2.flip(image, 0)
        # depth = cv2.flip(depth, 0)
        # image = cv2.flip(image, 1)
        # depth = cv2.flip(depth, 1)

        phantom = cv2.imread(phantom_file)
        phantom_resized = cv2.resize(phantom, (width, height))
        combined = cv2.hconcat([phantom_resized, image, depth])
        video.write(combined)
        pbar.update(1)

    video.release()
    cv2.destroyAllWindows()

    print(f"Video saved as '{output_video}'")

if __name__ == "__main__":
    dir2vid(savedir='../output/cpd_phantom1_PtoRb',
            name='registered_phantom1')