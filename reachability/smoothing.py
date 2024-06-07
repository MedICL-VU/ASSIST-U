from collections import defaultdict
import numpy as np


def smooth_path(vertices, edges, iterations=1, lambda_factor=0.5):
    # based on laplacian smoothing

    # Create a dictionary to store the neighbors of each vertex
    neighbors = defaultdict(set)
    for v1, v2 in edges:
        neighbors[v1].add(v2)
        neighbors[v2].add(v1)

    # Convert vertices dict to numpy array for easier manipulation
    vertex_positions = np.array([vertices[v] for v in sorted(vertices)])

    # Perform the smoothing iterations
    for _ in range(iterations):
        new_positions = np.copy(vertex_nodes)
        for i, pos in enumerate(vertex_positions):
            if i in neighbors:  # only adjust if the vertex has neighbors
                # Calculate the average position of neighbors
                neighbor_pos = np.mean([vertex_positions[n] for n in neighbors[i]], axis=0)
                # Update the vertex position based on the smoothing factor
                new_positions[i] = pos + lambda_factor * (neighbor_pos - pos)
        vertex_positions = new_position

    # Update the original vertices dictionary
    for i, v in enumerate(sorted(vertices)):
        vertices[v] = vertex_positions[i].tolist()

    return vertices


# Example usage
vertices = {0: [0, 0, 0], 1: [1, 0, 0], 2: [1, 1, 0], 3: [0, 1, 0]}
edges = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)]
smoothed_vertices = laplacian_smoothing(vertices, edges, iterations=10, lambda_factor=0.5)
print(smoothed_vertices)
