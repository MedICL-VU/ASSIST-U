import numpy as np
import open3d as o3d
import quaternion
from util.SocketCommand import send_command

def load_images(fname):
    # load line by line
    with open(fname, 'r') as f:
        lines = f.readlines()
        camera_poses = lines[4::2]


    poses = [line.split(" ", 9) for line in camera_poses]
    images = []
    for pose in poses:
        id = int(pose[0])
        rotation = np.asarray([float(pose[1]),float(pose[2]), float(pose[3]), float(pose[4])]) # w x y z
        position = np.asarray([float(pose[5]),float(pose[6]),float(pose[7])])
        camera_id = float(pose[8])
        image_name = pose[9][:-1] # strip newline
        images.append([id, position, rotation, camera_id, image_name])
    # sorted(images, key=lambda x: x[4])
    # return sorted(images, key=lambda x: x[4])
    return images

def load_mesh(fname, recenter=False):
    mesh = o3d.io.read_triangle_mesh(fname)
    # pcd = mesh.sample_points_poisson_disk(number_of_points=2000)
    mesh.compute_vertex_normals()  # Useful for better visualization
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.normals = mesh.vertex_normals
    center = pcd.get_center()
    if recenter:
        pcd.translate(-1* center)
    return pcd, mesh, center

def load_points(fname):
    # load line by line
    with open(fname, 'r') as f:
        lines = f.readlines()
        pointlines = lines[3::1]

    points = [line.split(" ", 9) for line in pointlines]
    pointlist = []
    for point in points:
        id = int(point[0])
        position = np.asarray([float(point[1]),float(point[2]),float(point[3])])
        color = np.asarray([float(point[4]),float(point[5]),float(point[6])])
        error = float(point[7])

        pointlist.append([id, position, color, error])

    return pointlist

def load_reconpoints(fname, recenter=False):
    points = load_points(fname)
    positions = np.asarray([point[1] for point in points])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions)
    center = pcd.get_center()
    if recenter:
        pcd.translate(-1 * center)
    return pcd, center

def get_rotation_matrices(rotations):
    # array-wise
    qs = quaternion.as_quat_array(rotations)
    matrices = quaternion.as_rotation_matrix(qs)
    return matrices

def calc_cam_center(images):
    # [(id, position, rotation, camera_id, image_name), ...]
    # COLMAP the local camera coordinate system of an image is defined in a way that
    # the X axis points to the right,
    # the Y axis to the bottom,
    # and the Z axis to the front as seen from the image.

    # coordinates of projection/camera center given by -R^t * T where R is rotation matrix, T is transpose (position)
    # [(id, cam center, rotation, camera_id, image_name), ...]
    rotations = np.asarray([image[2] for image in images])
    rotation_matrices = get_rotation_matrices(rotations)
    positions = np.asarray([image[1] for image in images])
    # positions[:,1] = -positions[:,1]
    for i, (position, rotation) in enumerate(zip(positions, rotation_matrices)):
        cam_center = np.dot(np.linalg.inv(rotation) , position)
        cam_center[0] = -cam_center[0]
        cam_center[1] = -cam_center[1]
        cam_center[2] = -cam_center[2]
        images[i][1] = cam_center
    return images

def adjust_image_center(images, center):
    # [(id, position, rotation, camera_id, image_name), ...]
    # recenters the camera positions according to registration steps # DEPENDS ON IF U DID IT DURING REG
    for image in images:
        image[1] = image[1] - center

    return images
def adjust_camera_scale(images, center, scale):
    # [(id, position, rotation, camera_id, image_name), ...]
    # rescales everything from center, mirrors registration process
    # roundabout (hopefully more reliable)way of doing positions + scale*(positions-center)
    positions = np.asarray([image[1] for image in images])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions)
    pcd.scale(scale, center=center)
    scaled_positions = np.asarray(pcd.points)
    for image, position in zip(images, scaled_positions):
        image[1] = position
    return images

def flip_camera_x(images, up_positions=None):
    # [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    for image in images:
        image[0][0] = -image[0][0]
        image[1][0] = -image[1][0]
        image[2][0] = -image[2][0]
    if up_positions is not None:
        for up in up_positions:
            up[0] = -up[0]
    return images, up_positions

def flip_camera_y(images, up_positions=False):
    # [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    for image in images:
        image[0][1] = -image[0][1]
        image[1][1] = -image[1][1]
        if up_positions:
            image[2][1] = -image[2][1]
    return images, up_positions

def flip_focal_y(images):
    # [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    for image in images:
        image[1][1] = -image[1][1]
    return images

def focal_up_to_rotation(images):
    # [(position, focal_point, up_points, image_name), ...]
    # for each position rotation up triplet compute a rotation quaternion
    # returns: [(position, focal_point, rotation, image_name), ...]
    rotations = []
    for image in images:
        position, focal, up = image[0], image[1], image[2]
        # compute rotation matrix
        z = focal - position
        z = z / np.linalg.norm(z)
        x = np.cross(up, z)
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)
        y = y / np.linalg.norm(y)
        R = np.column_stack((x, y, z))
        quat = quaternion.from_rotation_matrix(R)
        rotations.append([position, focal, quaternion.as_float_array(quat), image[3]])
    return rotations


def rotate_vector_by_quaternion(vector, rotation, invert= False):
    # Ensure the quaternion is normalized
    quat = quaternion.from_float_array(rotation)
    # quat = rotation
    if invert:
        quat = quat.inverse()
    quat_normalized = quat.normalized()
    # Convert the vector into a quaternion with zero real part
    vector_quat = quaternion.from_float_array([0.0] + list(vector))
    # Perform the rotation
    rotated_vector_quat = quat_normalized * vector_quat * quat_normalized.conjugate()
    # Convert the quaternion result back to a vector (ignore the real part)
    return quaternion.as_float_array(rotated_vector_quat)[1:]

def calculate_new_position(position, rotation_quaternion, direction_vector=np.array([0, 0, 0.25]), invert=False):
    # Rotate the direction vector by the quaternion
    rotated_vector = rotate_vector_by_quaternion(direction_vector, rotation_quaternion, invert)
    # Calculate new position by adding the rotated vector to the original position
    new_position = position + rotated_vector
    return new_position

def update_positions(full_positions, new_positions):
    # full positions: [(id, position, rotation, camera_id, image_name), ...]
    # new positions: [position, ...]
    # Update the positions in the full_positions list
    for i, position in enumerate(new_positions):
        full_positions[i][1] = position
    return full_positions

def update_rotations(full_positions, new_rotations):
    # full positions: [(id, position, rotation, camera_id, image_name), ...]
    # new positions: [position, ...]
    # Update the positions in the full_positions list
    for i, rotation in enumerate(new_rotations):
        quat = quaternion.from_rotation_matrix(rotation)
        full_positions[i][2] = quaternion.as_float_array(quat)
    return full_positions

def extract_euler_angles_from_quaternion(rotation, invert= False):
    quat = get_rotation_matrices(rotation)
    # quat = quaternion.from_float_array(rotation)
    # if invert:
    #     quat = quat.inverse() # try this for unity
    # quat = np.linalg.inv(quat)
    quat = quaternion.from_rotation_matrix(quat)
    # Ensure the quaternion is normalized
    # quat_normalized = quat.normalized()
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    euler_angles = quaternion.as_euler_angles(quat)
    # return euler_angles
    # Return angles in degrees
    return np.degrees(euler_angles)  # Return roll, pitch, yaw in degrees

def compute_focals(images):
    # [(id, position, rotation, camera_id, image_name), ...]
    # for each position rotation pair compute a unit vector movement in rotation direction, as well as camera roll
    # returns: [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    positions = []

    for image in images:
        position, rotation = image[1], image[2]
        # rotation[1] = -rotation[1]
        # rotation[2] = -rotation[2]
        focal_point = calculate_new_position(position, rotation, invert=False)

        #https://github.com/colmap/colmap/issues/1376
        #colmap (x,y,z,x°,y°,z°) = unity (x,-y,z,-x°,-y°,z°)
        # roll, pitch, yaw = extract_euler_angles_from_quaternion(rotation, invert=False)
        name = image[4].split('/')[1]
        positions.append((position, focal_point, rotation, name))
    return positions

def update_focals(full_focals, new_focals, up=None):
    # full focals: [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    # new focals: [position, ...]
    # Update the focal points in the full_focals list
    # print(f'full_focals len: {len(full_focals)}')
    # print(f'new_focals len : {len(new_focals)}')
    updated_focals = []
    for i, focal in enumerate(full_focals):
        if up is None:
            position = full_focals[i][0]
            transformed_focal = new_focals[i]
            orientation = full_focals[i][2]
            name = full_focals[i][3]
            new_focal = (position, transformed_focal, orientation, name)
            updated_focals.append(new_focal)
        else:
            position = full_focals[i][0]
            transformed_focal = new_focals[i]
            orientation = up[i]
            name = full_focals[i][3]
            new_focal = (position, transformed_focal, orientation, name)
            updated_focals.append(new_focal)
    return updated_focals

def find_norms_focal(images):
    # [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    # for each position rotation pair compute a unit vector movement in rotation direction, as well as camera roll
    # returns: [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    norms = []
    for image in images:
        position, focal = image[0], image[1]
        norm = np.linalg.norm(focal - position)
        norms.append(norm)
    return norms

def find_norms_up(images):
    # [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    # for each position rotation pair compute a unit vector movement in rotation direction, as well as camera roll
    # returns: [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    norms = []
    for image in images:
        position = image[0]
        up = image[2]
        norm = np.linalg.norm(up - position)
        norms.append(norm)
    return norms

if __name__ == '__main__':
    # load phantom pcd
    # recon_pcd, recon_mesh, recon_center = load_mesh('../data/soft3slowwet/meshed-poisson.ply', recenter=True)
    # # get center
    # # recon_center = recon_pcd.get_center()
    #
    # # load camera positions
    # camera_poses = load_images('../data/soft3slowwet/images.txt')
    # camera_poses = calc_cam_center(camera_poses)
    # camera_poses = adjust_image_center(camera_poses, recon_center)
    # camera_poses = adjust_camera_scale(camera_poses, recon_center, scale=5)
    #
    # positions = compute_focals(camera_poses)
    position = [-2.27335195, 18.13396321, 14.4140538 ]
    rotation =  [-0.507386, 0.840481, -0.120037, -0.147452]


    send_command(f'MoveCamera,{position[0]},{position[1]},{position[2]}')
    quat = quaternion.as_quat_array(rotation)
    R = quaternion.as_rotation_matrix(quat)
    R_t = np.linalg.inv(R)
    quat_2 = quaternion.from_rotation_matrix(R_t)
    # send_command(f'RotateCameraQuat,{quat.w},{quat.x},{quat.y},{quat.z}')
    # send_command(f'RotateCameraQuat,{quat_2.w},{quat_2.x},{quat_2.y},{quat_2.z}')

    angles = quaternion.as_euler_angles(quat_2)
    angles_deg = np.degrees(angles)
    print(angles_deg)

    send_command(f'RotateCameraQuat,{rotation[0]},{-rotation[1]},{-rotation[2]},{-rotation[3]}')

    #[ 25.93434878 138.31322849  93.49099976]
    pass

#position 461 [-6.28411459 -0.06755795 -7.49761367]
# position 361 [-2.27335195 18.13396321 14.4140538 ]
# rot 361 [-0.507386, 0.840481, -0.120037, -0.147452]

# pos 2274 [ 2.02309676,  7.73691849, 19.3038877 ]
# rot [0.952774, -0.292263, 0.0499467, 0.0656501]
# debugging params
# position [-11.30071532  39.94956768  30.84415942]
# focal [-11.13278016  40.34419754  29.94079623]
# roll pitch yaw [-246.94776979093007, 154.6037267846497, -59.23476880065692]
# name '002526_soft 3 med wet_manual_crop.png'