import numpy as np
from rendering import ureterPicker
import open3d as o3d
import scipy
import skeletor as sk
import trimesh
import copy
from util.mathfunctions import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    print ('Fixing mesh')
    fixed = sk.pre.fix_mesh(mesh, remove_disconnected=5)
    # print('Simplifying mesh') #requires blender
    # simple = sk.pre.simplify(fixed, 0.1)
    # print('Contracting mesh')
    # contracted = sk.pre.contract(simple, time_lim=120)

    # according to docs wavefront works for tubular
    swc = sk.skeletonize.by_wavefront(fixed, step_size=params['wavesize'], waves=params['wavecount'], progress=False)
    # swc = sk.skeletonize.by_edge_collapse(contracted, shape_weight=1, sample_weight=0.1)
    # cont = sk.pre.contract(mesh, iter_lim=100, epsilon=1e-06, SL=3)
    # swc = sk.skeletonize.by_vertex_clusters(mesh, sampling_dist=100)
    # swc = sk.skeletonize.by_teasar(simple, 100)

    # swc = sk.skeletonize.by_teasar(mesh, inv_dist=500)

    swc = sk.post.clean_up(swc, mesh)

    edgelist = []
    for edge in swc.edges:
        # get two points
        p0 = swc.vertices[edge[0]]
        p1 = swc.vertices[edge[1]]
        edgelist.append((p0,p1))
        edgelist.append((p1, p0))
    # print(f'{len(edgelist)//2} edges')

    if params['visualize']:
        # sample between edges
        skele_points = np.ndarray((0, 3))
        # for edge in swc.edges:
        for p0,p1 in edgelist:
            # get two points
            # p0 = swc.vertices[edge[0]]
            # p1 = swc.vertices[edge[1]]
            dist = np.linalg.norm(p0 - p1)
            num_points = int(np.floor(dist / 0.5))
            skele_points = np.append(skele_points, np.linspace(p0, p1, num_points), 0)
        skele_cloud = o3d.geometry.PointCloud()
        skele_cloud.points = o3d.utility.Vector3dVector(skele_points)
        draw_pc(skele_cloud, mesh_o3d.sample_points_uniformly(1000), 'test')

    return edgelist

def smooth_path(path, midpoint_iterations=5, smoothing_iterations=10):
    edges = [edges for direction, edges in path]
    vertices = [v0 for v0,v1 in edges]
    vertices.append(edges[-1][1])
    unique_vertices = np.unique(vertices)

    for _ in range(midpoint_iterations):
        # add midpoints to the list
        new_edges = []
        for idx, (v0,v1) in enumerate(edges):
            new_vertex = (v0 + v1)/2
            new_edges.append([v0, new_vertex])
            new_edges.append([new_vertex, v1])
        edges = new_edges

    # laplacian smoothing
    for _ in range(smoothing_iterations):
        new_edges = []
        new_edges.append(edges[0])
        # for each pair of edges
        connected = True
        for i in range(1, len(edges)):
            connected = np.all(new_edges[-1][1] == edges[i][0])
            if connected:
                v0 = edges[i-1][0]
                v1, v2 = edges[i]
                new_pos = np.mean([v0,v1,v2], axis=0)
                if connected:
                    new_edges[-1][1] = new_pos
                new_edges.append([new_pos, v2])

        # new_edges.append(edges[-1])
        edges = new_edges

    # readd the directional indexing
    return [[1,edge] for edge in edges]

def gen_positions(params):
    localbending = params['localbending']
    # gen ccamera points and angles
    modelpath = params['modelpath']
    if params['modelname'] in params['models']:
        ureter_coord = params['models'][params['modelname']]
    else:
        ureter_coord = ureterPicker.pickureter(modelpath)
    # ureter_coord = (23.840221383904932, -136.21629019416955, 804.6988184670687)
    print(f'Got ureter coords: {ureter_coord}')
    ureter_coord = np.asarray([ureter_coord])

    edges = get_edges(modelpath, params)
    edgelist = np.asarray(edges)

    # [25.8, -145, 807]

    # find closest point as starting location
    start = find_nearest(ureter_coord, edgelist[:, 0,:])

    # dfs edge paths and place in order
    # assumes no loops
    path = organize_paths(start, edgelist, params)
    path = smooth_path(path)

    if params['visualize']:
        mesh_o3d = o3d.io.read_triangle_mesh(modelpath)
        # sample between edges
        skele_points = np.ndarray((0, 3))
        # for edge in swc.edges:
        for direction, edge in path:
            p0, p1 = edge
            dist = np.linalg.norm(p0 - p1)
            num_points = int(np.floor(dist / params['samplestep']))
            skele_points = np.append(skele_points, np.linspace(p0, p1, num_points), 0)
        skele_cloud = o3d.geometry.PointCloud()
        skele_cloud.points = o3d.utility.Vector3dVector(skele_points)
        draw_pc(skele_cloud, mesh_o3d.sample_points_uniformly(10000), 'Selected edges')

    print(f'Selected {len(path)} edges for path gen')

    # for each edge traverse forwards and backwards recording views
    positions = []
    for direction, edge in path:
        # direction 1 is forward, -1 is backward
        v1, v2 = edge

        #sample points along edge
        dist = np.linalg.norm(v2 - v1)
        num_points = int(np.floor(dist / params['samplestep']))
        track = np.linspace(v1, v2, num_points)

        if direction == -1:
            continue
            # track = track[::-1]
        # forward view
        for i in range(0, len(track)-1):
            coords = sphere_gen(v1, v2, localbending, params['numpoints']) #still want same camera direction
            for j in range(len(coords)):
                positions.append((track[i], track[i] + coords[j]))  # coord, focal direction (nextpoint)

            # generate sample points up to localbending degrees off the v1->v2 vector

    return positions



def dfspath(path, edges, node, last, visited, globalbending, params):
    if node not in visited and len(path) < params['max_depth']:
        visited.add(node)
        v1,v2 = edges[node]
        # print(node, end=' ')  # Process the current node
        path.append([1, edges[node]])

        successors = find_successors(v2, edges[:, 0, :])

        success_count = 0
        for successor in successors:
            successor_vector = vectorize(edges[successor][0], edges[successor][1])

            if not np.linalg.norm(successor_vector) < 0.0000005 and angle(vectorize(v1, v2),
                                                                          successor_vector) < globalbending:
            # if not np.linalg.norm(successor_vector) < 0.0000005:
                path = dfspath(path, edges, successor, node, visited, globalbending, params)
                success_count +=1
        # if success_count == 0:
        #     path.append([-1, edges[node]])
        # path.append([-1, edges[last]])
    return path

def organize_paths(start, edges, params):
    path = []
    queue = []
    visited = set()
    mode=params['planmode']
    globalbending = params['globalbending']

    if mode == 'dfs':

        path = dfspath(path, edges, start, start, visited, globalbending, params)
        return path#[:-1]

    elif mode =='bfs':
        path.append([1,edges[start]])
        queue.append(edges[start])
        while len(queue) > 0:
            v1, v2 = queue.pop(0)
            successors = find_successors(v2, edges[:, 0, :])  # returns indices
            for successor in successors:
                successor_vector = vectorize(edges[successor][0], edges[successor][1])
                if mode == 'unfiltered':
                    if successor not in visited:
                        # if not length 0, angle is valid, and not seen before
                        path.append([1,edges[successor]])
                        queue.append(edges[successor])
                        visited.add(successor)
                elif not np.linalg.norm(successor_vector) < 0.0000005 and angle(vectorize(v1, v2), successor_vector) < globalbending and successor not in visited:
                    # if not length 0, angle is valid, and not seen before
                    path.append([1,edges[successor]])
                    queue.append(edges[successor])
                    visited.add(successor)
                else:
                    pass
    return path



def find_nearest(point, edgelist):
    distances = scipy.spatial.distance.cdist(point, edgelist)
    index = np.argmin(distances)
    return index

def find_successors(point, edgelist, tolerances=0.1):
    distances = scipy.spatial.distance.cdist([point], edgelist)
    successors = np.argwhere(distances < tolerances)[:,1]
    return successors

def occlusion_check(edge_list):
    pass

# def angle_check(center, sphere_coords, degrees):
#     distances = scipy.spatial.distance.cdist([center], sphere_coords, 'cosine')
#
#     return angles

def sphere_gen(v1,v2, local_bending, points):
    # points to sample around axis

    num_points = points
    unit_center = unit_vector(vectorize(v1, v2))

    if local_bending == 0:
        return [unit_center]

    sphere_coords, _ = fibonacci_sphere2(points, unit_center, local_bending)

    return sphere_coords


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

def fibonacci_sphere2(samples, direction, angle):
    num_points = samples
    cap_size = 2 * np.pi * 1 * (1 - np.cos(np.deg2rad(angle)))
    sphere_size = 4 * np.pi * 1
    ratio = sphere_size / cap_size
    sphere_coords = fibonacci_sphere(int(num_points * ratio))
    angles = angle_matrix(direction, sphere_coords)
    inside = sphere_coords[angles<=angle]
    if len(inside) <1:
        inside = np.array([direction])
    outside = sphere_coords[angles > angle]
    return inside, outside

def viz_sphere(points1, points2):
    # Creating a new figure
    fig = plt.figure(figsize=(10,10))

    # Adding a subplot with 3D capability
    ax = fig.add_subplot(111, projection='3d')

    # Extracting x, y, and z coordinates from the list of points
    x_coords = [point[0] for point in points1]
    y_coords = [point[1] for point in points1]
    z_coords = [point[2] for point in points1]
    x2_coords = [point[0] for point in points2]
    y2_coords = [point[1] for point in points2]
    z2_coords = [point[2] for point in points2]

    # Plotting the points
    ax.scatter(x_coords, y_coords, z_coords, c='blue')
    ax.scatter(x2_coords, y2_coords, z2_coords, c='red')

    # Setting labels for the axes
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')

    # Displaying the plot
    plt.show()

def main():
    '''
    180 100%
    120 60
    90 50%
    60
    45
    30
    15
    10
    5
    0
    Returns
    -------

    '''
    inside, outside = fibonacci_sphere2(9, np.array([1,0,0]), 30)
    print('Num inside: ', len(inside))
    viz_sphere(inside, outside)

if __name__ == "__main__":
    main()