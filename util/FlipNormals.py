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
input_obj = '../data/3dmodels/registered_phantom_test.obj'
output_obj = '../data/3dmodels/registered_phantom_test_flip.obj'
flip_normals(input_obj, output_obj)

