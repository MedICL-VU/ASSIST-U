import meshio


mesh = meshio.read('../data/cao1_3_19_25_model3.stl')

# Flip the x-coordinates
mesh.points[:, 0] = -mesh.points[:, 0]

meshio.write('../data/cao1_3_19_25_model3.obj', mesh)