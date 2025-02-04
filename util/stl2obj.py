import meshio


mesh = meshio.read('../data/soft3slowwet/registration2/phantom1_centered.stl')

# Flip the x-coordinates
mesh.points[:, 0] = -mesh.points[:, 0]

meshio.write('../data/soft3slowwet/registration2/phantom1_centered.obj', mesh)