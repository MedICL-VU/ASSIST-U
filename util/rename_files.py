import os
from tqdm import tqdm

def rename_files(directory_path, new_name, extension):
    # List all the relevant files
    files = [f for f in os.listdir(directory_path) if f.endswith(f"_{extension}.png") and f.split('_')[0].isdigit()]

    for filename in tqdm(files, desc="Renaming files"):
        # Splitting the filename into parts
        parts = filename.split("_")
        # Constructing the new filename
        new_filename = f"{parts[0]}_{new_name}_{extension}.png"
        # Renaming the file
        os.rename(os.path.join(directory_path, filename), os.path.join(directory_path, new_filename))

if __name__ == "__main__":
    # Replace 'directory_path' with the path to your directory and 'newname' with your desired name
    # system = 'Patient3Left'
    dirs = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    extension = 'render'
    for system in dirs:
        directory_path = f'../output/output_fine_multiline/{system}/render'  # e.g., 'C:/images'

        new_name = system  # e.g., 'example'

        rename_files(directory_path, new_name, extension)
