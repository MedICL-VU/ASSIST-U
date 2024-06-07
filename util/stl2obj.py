import meshio


mesh = meshio.read('../data/3dmodels/smoothed_arpah.stl')
meshio.write('../data/3dmodels/smoothed_arpah.obj', mesh)