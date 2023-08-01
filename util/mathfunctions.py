import numpy as np

def angle(v1, v2):
    # from https://stackoverflow.com/a/13849249
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.rad2deg(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

def angle_matrix(v1, v2): # vector to matrix
    return np.rad2deg(np.arccos(np.clip(np.dot(v2, v1), -1.0, 1.0)))

def unit_vector(v):
    return v/np.linalg.norm(v)

def vectorize(v1,v2):
    return v2-v1