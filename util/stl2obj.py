import meshio


mesh = meshio.read('../data/3dmodels/registered_phantom_test.stl')
meshio.write('../data/3dmodels/registered_phantom_test.obj', mesh)