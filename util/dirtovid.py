import os
import cv2

def dir2vid(savedir, name):
    # Path to the folder containing the images
    image_folder = f'{savedir}/{name}/render/'
    depth_folder = f'{savedir}/{name}/depth/'

    # Output video file name
    output_video = f'{savedir}/{name}.mp4'

    # Frame rate (frames per second) for the output video
    frame_rate = 30  # You can adjust this as needed

    # Get the list of image files in the folder and sort them alphabetically
    image_files = sorted([os.path.join(image_folder, file) for file in os.listdir(image_folder) if file.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))])
    depth_files = sorted([os.path.join(depth_folder, file) for file in os.listdir(depth_folder) if file.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))])

    if not image_files:
        print("No image files found in the folder.")
        exit()

    # Open the first image to get its dimensions
    first_image = cv2.imread(image_files[0])
    height, width, layers = first_image.shape

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can change the codec as needed
    video = cv2.VideoWriter(output_video, fourcc, frame_rate, (width * 2, height))

    for image_file, depth_file in zip(image_files, depth_files):
        image = cv2.imread(image_file)
        depth = cv2.imread(depth_file)
        combined = cv2.hconcat([image, depth])
        video.write(combined)

    video.release()
    cv2.destroyAllWindows()

    print(f"Video saved as '{output_video}'")

if __name__ == "__main__":
    dir2vid(savedir='../output/arpah_unity_smooth', name='arpah_decimated')