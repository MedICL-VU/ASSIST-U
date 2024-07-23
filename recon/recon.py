import numpy as np
import open3d as o3d
import quaternion

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
    return sorted(images, key=lambda x: x[4])

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

    # caoordinates of projection/camera center given by -R^t * T where R is rotation matrix, T is transpose (position)
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

def flip_camera_x(images):
    # [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    for image in images:
        image[0][0] = -image[0][0]
        image[1][0] = -image[1][0]
    return images

def rotate_vector_by_quaternion(vector, rotation, invert= False):
    # Ensure the quaternion is normalized
    quat = quaternion.from_float_array(rotation)
    if invert:
        quat = quat.inverse()
    quat_normalized = quat.normalized()
    # Convert the vector into a quaternion with zero real part
    vector_quat = quaternion.from_float_array([0.0] + list(vector))
    # Perform the rotation
    rotated_vector_quat = quat_normalized * vector_quat * quat_normalized.conjugate()
    # Convert the quaternion result back to a vector (ignore the real part)
    return quaternion.as_float_array(rotated_vector_quat)[1:]

def calculate_new_position(position, rotation_quaternion, direction_vector=np.array([0, 0, 1]), invert=False):
    # Rotate the direction vector by the quaternion
    rotated_vector = rotate_vector_by_quaternion(direction_vector, rotation_quaternion, invert)
    # Calculate new position by adding the rotated vector to the original position
    new_position = position + rotated_vector
    return new_position

def extract_euler_angles_from_quaternion(rotation, invert= False):
    quat = get_rotation_matrices(rotation)
    # quat = quaternion.from_float_array(rotation)
    # if invert:
    #     quat = quat.inverse() # try this for unity
    rotation_inv = np.linalg.inv(quat)
    quat = quaternion.from_rotation_matrix(rotation_inv)
    # Ensure the quaternion is normalized
    quat_normalized = quat.normalized()
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    euler_angles = quaternion.as_euler_angles(quat_normalized)
    # Return angles in degrees
    return np.degrees(euler_angles)  # Return roll, pitch, yaw in degrees


def compute_focals(images):
    # [(id, position, rotation, camera_id, image_name), ...]
    # for each position rotation pair compute a unit vector movement in rotation direction, as well as camera roll
    # returns: [(position, focal_point, [roll, pitch, yaw], image_name), ...]
    positions = []

    for image in images:
        position, rotation = image[1], -image[2]
        focal_point = calculate_new_position(position, rotation, invert=False)

        #https://github.com/colmap/colmap/issues/1376
        #colmap (x,y,z,x°,y°,z°) = unity (x,-y,z,-x°,-y°,z°)
        roll, pitch, yaw = extract_euler_angles_from_quaternion(rotation, invert=False)
        name = image[4].split('/')[1]
        positions.append((position, focal_point, [-1*roll,-1* pitch, yaw], name))
    return positions


if __name__ == '__main__':
    # load phantom pcd
    recon_pcd, recon_mesh, recon_center = load_mesh('../data/soft3slowwet/meshed-poisson.ply', recenter=True)
    # get center
    # recon_center = recon_pcd.get_center()

    # load camera positions
    camera_poses = load_images('../data/soft3slowwet/images.txt')
    camera_poses = calc_cam_center(camera_poses)
    camera_poses = adjust_image_center(camera_poses, recon_center)
    camera_poses = adjust_camera_scale(camera_poses, recon_center, scale=5)

    positions = compute_focals(camera_poses)
    pass


# debugging params
# position [-11.30071532  39.94956768  30.84415942]
# focal [-11.13278016  40.34419754  29.94079623]
# roll pitch yaw [-246.94776979093007, 154.6037267846497, -59.23476880065692]
# name '002526_soft 3 med wet_manual_crop.png'