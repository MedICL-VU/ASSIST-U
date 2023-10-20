import os
import cv2

# Path to the folder containing the images
image_folder = '../output/manualsegmentation1'

# Output video file name
output_video = 'output_video.mp4'

# Frame rate (frames per second) for the output video
frame_rate = 30  # You can adjust this as needed

# Get the list of image files in the folder and sort them alphabetically
image_files = sorted([os.path.join(image_folder, file) for file in os.listdir(image_folder) if file.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))])

if not image_files:
    print("No image files found in the folder.")
    exit()

# Open the first image to get its dimensions
first_image = cv2.imread(image_files[0])
height, width, layers = first_image.shape

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can change the codec as needed
video = cv2.VideoWriter(output_video, fourcc, frame_rate, (width, height))

for image_file in image_files:
    image = cv2.imread(image_file)
    video.write(image)

video.release()
cv2.destroyAllWindows()

print(f"Video saved as '{output_video}'")
