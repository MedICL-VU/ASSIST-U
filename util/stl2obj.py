import meshio


mesh = meshio.read('../data/soft3slowwet/reverseReg2/phantom1_registered_reverse.stl')

# Flip the x-coordinates
mesh.points[:, 0] = -mesh.points[:, 0]

meshio.write('../data/soft3slowwet/reverseReg2/phantom1_registered_reverse.obj', mesh)