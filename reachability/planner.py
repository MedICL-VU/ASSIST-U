import numpy as np
from rendering import ureterPicker
import open3d as o3d
import scipy
import skeletor as sk
import trimesh
import copy
from util.mathfunctions import *
import matplotlib.pyplot as plt

#edgelist

def draw_pc(source, target, title=''):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # [0.039, 0.039, 0.039]
    # []
    source_temp.paint_uniform_color([0.812, 0.039, 0.039])
    target_temp.paint_uniform_color([0.812, 0.682, 0.439])
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name=title)


def get_edges(modelpath, params):
    # load our stl and convert into triangle mesh for sk
    mesh = trimesh.load(modelpath)
    mesh_o3d = o3d.io.read_triangle_mesh(modelpath)

    # according to docs wavefront works for tubular
    swc = sk.skeletonize.by_wavefront(mesh, step_size=params['wavesize'], waves=params['wavecount'], progress=False)
    # swc = sk.skeletonize.by_edge_collapse(mesh, shape_weight=0.1, sample_weight=0.01)
    # cont = sk.pre.contract(mesh, iter_lim=100, epsilon=1e-06, SL=3)
    # swc = sk.skeletonize.by_vertex_clusters(mesh, sampling_dist=100)

    # swc = sk.skeletonize.by_teasar(mesh, inv_dist=500)

    swc = sk.post.clean_up(swc, mesh)

    edgelist = []
    for edge in swc.edges:
        # get two points
        p0 = swc.vertices[edge[0]]
        p1 = swc.vertices[edge[1]]
        edgelist.append((p0,p1))
        edgelist.append((p1, p0))
    print(f'{len(edgelist)//2} edges')

    if params['visualize']:
        # sample between edges
        skele_points = np.ndarray((0, 3))
        for edge in swc.edges:
            # get two points
            p0 = swc.vertices[edge[0]]
            p1 = swc.vertices[edge[1]]
            dist = np.linalg.norm(p0 - p1)
            num_points = int(np.floor(dist / 0.5))
            skele_points = np.append(skele_points, np.linspace(p0, p1, num_points), 0)
        skele_cloud = o3d.geometry.PointCloud()
        skele_cloud.points = o3d.utility.Vector3dVector(skele_points)
        draw_pc(skele_cloud, mesh_o3d.sample_points_uniformly(1000), 'test')

    return edgelist

def gen_positions(modelpath, params):
    localbending = params['localbending']
    globalbending = params['globalbending']
    # gen ccamera points and angles
    modelpath = modelpath
    # ureter_coord = ureterPicker.pickureter(modelpath)
    ureter_coord = params['models'][params['modelname']]
    # ureter_coord = (23.840221383904932, -136.21629019416955, 804.6988184670687)
    print(f'Got ureter coords: {ureter_coord}')
    ureter_coord = np.asarray([ureter_coord])

    edges = get_edges(modelpath, params)
    edgelist = np.asarray(edges)

    # [25.8, -145, 807]

    # find closest point as starting location
    start = find_nearest(ureter_coord, edgelist[:, 0,:])

    # bfs edge paths and place in order
    # assumes no loops
    path = organize_paths(start, edgelist, globalbending)

    print(f'Selected {len(path)} edges for path gen')

    # for each edge traverse forwards and backwards recording views
    positions = []
    for edge in path:
        v1, v2 = edge

        #sample points along edge
        dist = np.linalg.norm(v1 - v2)
        num_points = int(np.floor(dist / 0.5))
        track = np.linspace(v1, v2, num_points)

        # forward view
        for i in range(0, len(track)-1):
            coords = sphere_gen(v1, v2, localbending)
            for j in range(len(coords)):
                positions.append((track[i], coords[j]))  # coord, focalpoint (nextpoint)

            # generate sample points up to localbending degrees off the v1->v2 vector

    return positions

def organize_paths(start, edges, globalbending):
    path = []
    queue = []
    visited = set()
    # v1, v2 = edges[start]
    # successors = find_successors(v2, edges[:,0,:]) # returns indices
    # for successor in successors:
    #     if angle(vectorize(v1,v2), vectorize(edges[successor][0], edges[successor][1])) < globalbending:
    #         queue += successor
    path.append(edges[start])
    queue.append(edges[start])
    visited.add(start)
    while len(queue) > 0:
        v1, v2 = queue.pop(0)
        successors = find_successors(v2, edges[:, 0, :])  # returns indices
        for successor in successors:
            if angle(vectorize(v1, v2), vectorize(edges[successor][0], edges[successor][1])) < globalbending and successor not in visited:
                path.append(edges[successor])
                queue.append(edges[successor])
                visited.add(successor)
    return path



def find_nearest(point, edgelist):
    distances = scipy.spatial.distance.cdist(point, edgelist)
    index = np.argmin(distances)
    return index

def find_successors(point, edgelist, tolerances=0.1):
    distances = scipy.spatial.distance.cdist([point], edgelist)

    return np.argwhere(distances < tolerances)[:,1]

def smooth_path(edges):
    pass

def occlusion_check(edge_list):
    pass

# def angle_check(center, sphere_coords, degrees):
#     distances = scipy.spatial.distance.cdist([center], sphere_coords, 'cosine')
#
#     return angles

def sphere_gen(v1,v2, local_bending):
    # points to sample around axis
    num_points = 10
    ratio = local_bending/180
    sphere_coords = fibonacci_sphere(int(num_points*(1/ratio)))
    unit_center = unit_vector(vectorize(v1,v2))
    angles = angle_matrix(unit_center, sphere_coords)
    return sphere_coords[angles<local_bending]*vectorize(v1,v2)

def fibonacci_sphere(samples):
    # https://gist.github.com/Seanmatthews/a51ac697db1a4f58a6bca7996d75f68c

    ga = (3 - np.sqrt(5)) * np.pi # golden angle

    # Create a list of golden angle increments along tha range of number of points
    theta = ga * np.arange(samples)

    # Z is a split into a range of -1 to 1 in order to create a unit circle
    # range_min = 2*ratio
    z = np.linspace(1/samples-1, 1-1/samples, samples)

    # a list of the radii at each height step of the unit circle
    radius = np.sqrt(1 - z * z)

    # Determine where xy fall on the sphere, given the azimuthal and polar angles
    y = radius * np.sin(theta)
    x = radius * np.cos(theta)

    coords = np.array((x,y,z))

    return coords.T

def main():
    fibonacci_sphere(1000)

if __name__ == "__main__":
    main()