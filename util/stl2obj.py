import meshio

# convert STL to OBJ
# if needed, flip x axis

name = '31R'
mesh = meshio.read(f'../data/mdephantom/models/pt{name}.stl')
# mesh.points[:, 0] = -mesh.points[:, 0]
meshio.write(f'../data/mdephantom/models/pt{name}.obj', mesh)


mesh = meshio.read(f'../data/mdephantom/models/{name}_register_reverse.stl')
# mesh.points[:, 0] = -mesh.points[:, 0]
meshio.write(f'../data/mdephantom/models/{name}_register_reverse.obj', mesh)

