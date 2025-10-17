import nibabel as nib
import numpy as np
from skimage import measure
import trimesh
import os
import meshio

def nii_to_stl(input_path, output_path, label_value=1):
    # Load the NIfTI file
    nii = nib.load(input_path)
    volume = nii.get_fdata()

    # Optional: threshold for binary masks (if not already binary)
    binary = (volume == label_value).astype(np.uint8)

    # Run marching cubes
    verts, faces, normals, values = measure.marching_cubes(binary, level=0.5)

    # Transform to world coordinates using the affine
    verts = nib.affines.apply_affine(nii.affine, verts)

    # Create mesh
    mesh = trimesh.Trimesh(vertices=verts, faces=faces)

    # Save to STL
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    mesh.export(output_path)
    print(f"Saved STL to {output_path}")

def flip_normals(input_file, output_file):
    # Read the input OBJ file
    with open(input_file, 'r') as file:
        lines = file.readlines()

    # Container for the modified lines
    new_lines = []

    for line in lines:
        if line.startswith('f'):  # face data
            parts = line.split()
            # Reverse the order of vertices in the face definition (excluding the 'f' part)
            flipped = parts[0] + ' ' + ' '.join(parts[:0:-1])
            new_lines.append(flipped + '\n')
        else:
            # Copy other lines unchanged
            new_lines.append(line)

    # Write the output OBJ file
    with open(output_file, 'w') as file:
        file.writelines(new_lines)

# Example usage
if __name__ == "__main__":
    segmentation_dir = '/media/david/Extreme SSD/segmentation/'
    segmentation_list = sorted(os.listdir(segmentation_dir))
    for seg in segmentation_list:
        input_nii = os.path.join(segmentation_dir, seg)
        output_stl = os.path.join(segmentation_dir, seg.replace('.nii.gz', '.stl'))
        nii_to_stl(input_nii, output_stl)

        mesh = meshio.read(output_stl)
        output_obj = output_stl.replace('.stl', '.obj')
        # Flip the x-coordinates
        mesh.points[:, 0] = -mesh.points[:, 0]
        meshio.write(output_obj, mesh)
        flip_normals(output_obj, output_obj.replace('.obj', '_flipped.obj'))

