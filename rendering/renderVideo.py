import os
import math
import numpy as np
from PIL import Image

from recon.recon import find_norms_focal
from rendering import render
from reachability import planner
from reachability import reachability
from tqdm import tqdm

# noinspection PyUnresolvedReferences
import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from util.dirtovid import dir2vid
from recon import recon

# RenderingContextOpenGL2
#   RenderingCore
#   RenderingFreeType
#   RenderingGL2PSOpenGL2
#   RenderingOpenGL2

#vtkOpenGLRenderer
#vtkSquencePass
#vtkRenderPassCollection
#vtkShadowMapPass
#vtkOpaquePass
#vtkCameraPass
from vtkmodules.vtkRenderingOpenGL2 import (
    vtkOpenGLRenderer,
    vtkSequencePass,
    vtkRenderPassCollection,
    vtkShadowMapPass,
    vtkOpaquePass,
    vtkCameraPass
)
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkLight,
    vtkCamera,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer,
    vtkWindowToImageFilter,
)

from vtkmodules.vtkFiltersCore import (
    vtkSmoothPolyDataFilter,
    vtkPolyDataNormals
)

from vtkmodules.util.numpy_support import vtk_to_numpy

from vtkmodules.vtkCommonCore import vtkMath
from vtkmodules.vtkCommonDataModel import vtkVector3d

from vtkmodules.vtkImagingCore import (
    vtkImageShiftScale,
)
from tqdm import tqdm
from util.SocketCommand import send_command

def stitch_dmap(rgb_outputs, depth_outputs, x,y):
    dmap = np.zeros((x,y))
    for rgb, depth in zip(rgb_outputs, depth_outputs):
        # where rgb != 255,255,255
        # quick_sum = np.sum(rgb, axis=2)
        # image_mask = quick_sum != 255*3
        image_mask = (rgb != [0,0,255]).any(axis=2)
        # image_mask2 = (rgb != [0, 0, 254]).all(axis=2)
        dmap_mask = dmap == 0
        # where dmap still 0, update with latest depth where rgb is positive
        dmap = np.where(np.logical_and(image_mask, dmap_mask),depth, dmap)

    return dmap


def logistic_rescale(dmap, steepness=10, midpoint=0.5):
    """
    Applies a logistic transformation to the depth map to increase intensity variation
    more in the near plane than the far plane.

    :param dmap: Depth map, scaled between 0 and 255.
    :param steepness: Controls the steepness of the logistic curve.
    :param midpoint: The value at which the logistic curve's midpoint occurs.
    :return: Rescaled depth map.
    """
    # Normalize the depth map to range [0, 1]
    normalized_dmap = dmap / 255.0

    # Apply the logistic function
    logistic_dmap = 1 / (1 + np.exp(-steepness * (normalized_dmap - midpoint)))

    # Scale back to range [0, 255] if needed
    rescaled_dmap = 255 * logistic_dmap

    return rescaled_dmap

def save_unity(id, coord, focal, savedir, modelname, rotation=None, up=None):

    # currently assumes model already loaded
    send_command(f'MoveCamera,{coord[0]},{coord[1]},{coord[2]}')
    # print(f'MoveCamera,{coord[0]},{coord[1]},{coord[2]}')
    # send_command(f'RotateCameraLook,{-focal[0]},{focal[1]},{focal[2]}')
    if rotation is not None:
        # send_command(f'RotateCamera,{rotationw},{rotationx},{rotationy}')
        # send_command(f'RotateCameraQuat,{rotationw},{-rotationx},{-rotationy},{-rotationz}')
        rotationw = rotation[0]
        rotationx = rotation[1]
        rotationy = rotation[2]
        rotationz = rotation[3]
        send_command(f'RotateCameraQuat,{rotationw},{rotationx},{-rotationy},{rotationz}')

    elif up is not None:
        # print(f'RotateCameraLook,{focal[0]},{focal[1]},{focal[2]}, {up[0]},{up[1]},{up[2]}')
        send_command(f'RotateCameraLook,{focal[0]},{focal[1]},{focal[2]}, {up[0]},{up[1]},{up[2]}')
        # send_command(f'RotateCameraLook,{focal[0]},{focal[1]},{focal[2]}, {0},{1},{0}')
    else:
        send_command(f'RotateCameraLook,{focal[0]},{focal[1]},{focal[2]}')

    fpath = os.path.join(os.getcwd(), savedir, modelname)
    rendername= os.path.join(fpath,'render', id+'_render.png')
    send_command(f'Screenshot,{rendername},Normal')
    depthname = os.path.join(fpath, 'depth', id + '_dmap.png')
    send_command(f'Screenshot,{depthname},Depth')
    rawname = os.path.join(fpath, 'raw', id + '_dmap.exr')
    send_command(f'ScreenshotFull,{rawname},Depth')

def save_vtk(id, coord, focal, savedir, modelpath, modelname, save, cam_roll=None):
    # string, tuple, tuple
    colors = vtkNamedColors()
    colors.SetColor('100W Tungsten', [255, 214, 170, 255])
    colors.SetColor('Full Green', [0,255,0,255])
    colors.SetColor('Full White', [255, 255, 255, 255])
    colors.SetColor('Full Blue', [0, 0, 255, 255])
    colors.SetColor('Kidney', [250,193,175,255])

    # Create a reader
    filename = modelpath
    reader = vtkSTLReader()
    reader.SetFileName(filename)

    smoothFilter = vtkSmoothPolyDataFilter()
    smoothFilter.SetInputConnection(reader.GetOutputPort())
    smoothFilter.SetNumberOfIterations(15)
    smoothFilter.SetRelaxationFactor(0.1)
    smoothFilter.FeatureEdgeSmoothingOff()
    # smoothFilter.FeatureEdgeSmoothingOn()
    # smoothFilter.BoundarySmoothingOff()
    smoothFilter.BoundarySmoothingOn()
    smoothFilter.Update()

    normalGenerator = vtkPolyDataNormals()
    normalGenerator.SetInputConnection(smoothFilter.GetOutputPort())
    normalGenerator.ComputePointNormalsOn()
    normalGenerator.ComputeCellNormalsOn()
    normalGenerator.Update()

    # Create a mapper and actor
    mapper = vtkPolyDataMapper()
    # mapper.SetInputConnection(reader.GetOutputPort())
    mapper.SetInputConnection(normalGenerator.GetOutputPort())

    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetSpecular(0.2)
    actor.GetProperty().SetSpecularPower(2)
    actor.GetProperty().SetAmbient(0)
    actor.GetProperty().SetDiffuse(5)
    actor.GetProperty().SetColor(colors.GetColor3d('Kidney'))
    # actor.GetProperty().SetInterpolationToPBR()
    # actor.GetProperty().SetColor(colors.GetColor3d('darksalmon'))

    camera = vtkCamera()
    camera.SetPosition(coord[0], coord[1], coord[2])
    camera.SetFocalPoint(focal[0], focal[1], focal[2])
    # camera.SetViewAngle(2)
    if cam_roll is not None:
        camera.SetRoll(cam_roll)

    # Create a renderer, render window, and interactor
    # renderer = vtkRenderer()
    renderer = vtkOpenGLRenderer()
    seq = vtkSequencePass()
    passes = vtkRenderPassCollection()
    shadows = vtkShadowMapPass()

    passes.AddItem(shadows.GetShadowMapBakerPass())
    passes.AddItem(shadows)

    opaque =vtkOpaquePass()
    passes.AddItem(opaque)

    seq.SetPasses(passes)

    cameraP = vtkCameraPass()
    cameraP.SetDelegatePass(seq)

    renderer.SetPass(cameraP)

    renderer.SetActiveCamera(camera)
    renderer.AutomaticLightCreationOff()
    renderer.RemoveAllLights()
    # renderer.TwoSidedLightingOff()

    light1 = vtkLight()
    light1.SetLightTypeToHeadlight()
    light1.SetPosition(coord[0], coord[1], coord[2])
    light1.SetFocalPoint(focal[0], focal[1], focal[2])

    light1.SetColor(colors.GetColor3d('100W Tungsten'))
    light1.SetAttenuationValues(0.8, 0.2, 0)
    # light1.SetAttenuationValues(0.8,1,0)
    # print(light1.GetConeAngle())
    # light1.SetConeAngle(30)
    # light1.SetColor(colors.GetColor3d('White'))
    light1.SetIntensity(10)
    light1.PositionalOn()
    # light1.SetIntensity(2)
    renderer.AddLight(light1)
    light1.SwitchOn()

    x = y = 512

    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetWindowName('Camera')
    renderWindow.SetSize(x, y)
    renderWindow.Render()

    # Add the actor to the scene
    renderer.AddActor(actor)
    renderer.SetBackground(colors.GetColor3d('Full Blue'))
    # renderer.ResetCameraClippingRange()
    # camera.SetClippingRange(1, 10)

    if save:
        w2if = vtkWindowToImageFilter()
        w2if.SetInput(renderWindow)
        w2if.SetInputBufferTypeToRGB()
        w2if.ReadFrontBufferOn()
        # w2if.ReadFrontBufferOff()
        w2if.Update()
        writer = vtkPNGWriter()
        writer.SetFileName(os.path.join(savedir, modelname, 'render', f'{id}_render.png'))
        writer.SetInputConnection(w2if.GetOutputPort())
        writer.Write()

        # depth

        # manual depth 2 - stitching
        # ratios = [(0.1,1), (1,5), (5,10), (10,50),(50,100), (100,500), (500,1000)]
        ratios = [(1e-5,0.1),
                  (0.1,0.5),
                  (0.5,1),
                  (1,5),
                  (5,10),
                  (10,15),
                  (15,20),
                  (20, 25),
                  (25, 30),
                  (30, 50),
                  (50,100),
                  (100,250),
                  (100,500),]
                  # (500,1000),]
        rgb_outputs = []
        depth_outputs = []
        for (near, far) in ratios:
            camera.SetClippingRange(near, far)
            w2if.SetInput(renderWindow)
            w2if.SetInputBufferTypeToZBuffer()
            w2if.Update()
            scalars = w2if.GetOutput().GetPointData().GetScalars()
            image = vtk_to_numpy(scalars)
            image = image.reshape(x, y)
            image = -1 * 2. * near * far / ((image - 0.5) * 2. * (far - near) - near - far) # without parallel projection
            # image *= -255
            depth_outputs.append(image)

            w2if.SetInputBufferTypeToRGB()
            w2if.Update()
            scalars = w2if.GetOutput().GetPointData().GetScalars()
            image = vtk_to_numpy(scalars)
            image = image.reshape(x,y,-1)
            rgb_outputs.append(image)

        # for ratio, rgb, depth in zip(ratios, rgb_outputs, depth_outputs):
        #     im = Image.fromarray(rgb)
        #     im = im.convert('RGB')
        #     im.save(os.path.join(savedir, modelname, f'{id}_{ratio}_rgb.png'))
        #
        #     im = Image.fromarray(depth)
        #     im = im.convert('RGB')
        #     im.save(os.path.join(savedir, modelname, f'{id}_{ratio}_dslice.png'))
        dmap = stitch_dmap(rgb_outputs, depth_outputs, x, y)
        dmap = np.flipud(dmap)
        im = Image.fromarray(dmap)
        # im = im.convert('F')
        im.save(os.path.join(savedir, modelname, 'raw', f'{id}_dmap_raw.tiff'))

        dmap = 255.0 * (dmap - dmap.min()) / dmap.ptp()
        # invert dmap
        # dmap = 255. - dmap
        # dmap = logistic_rescale(dmap,steepness=2,  midpoint=0.75)

        im = Image.fromarray(dmap)
        im = im.convert('L')
        im.save(os.path.join(savedir, modelname, 'depth', f'{id}_dmap.png'))


        # print(camera.GetParallelProjection())

        # renderWindow.Finalize()
        with open(os.path.join(savedir, modelname)+'.txt', 'a') as file:
            file.write(f'{id}_render.png {" ".join([str(num) for num in coord])} {" ".join([str(num) for num in focal])} {" ".join([str(num) for num in camera.GetOrientationWXYZ()])}\n')
    else:
        renderWindowInteractor = vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        renderWindowInteractor.Initialize()
        renderWindowInteractor.Start()

def render(params):
    if not os.path.isdir(os.path.join(params['savedir'], params['modelname'])):
        os.makedirs(os.path.join(params['savedir'], params['modelname']))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'render'))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'depth'))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'raw'))

    # generate camera positions
    positions = planner.gen_positions(params)


    count = 0
    # pbar = tqdm(positions)
    for coord, focalpoint in positions:
        if params['renderer'] == 'unity' and params['save']:
            # move camera here
            # calculate rotation
            # get dir
            # save view
            # coord[1] = coord[1] + 2
            # focalpoint[1] = focalpoint[1]+2
            save_unity(f'{count:05}', coord, focalpoint, params['savedir'], params['modelname'])
        else:
            save_vtk(f'{count:05}', coord, focalpoint, params['savedir'], params['modelpath'], params['modelname'], params['save'])
        count += 1
        #break

    # dir2vid(params['savedir'], params['modelname'])

    return

def render_registered_pair_PtoR(params):
    if not os.path.isdir(os.path.join(params['savedir'], params['modelname'])):
        os.makedirs(os.path.join(params['savedir'], params['modelname']))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'render'))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'depth'))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'raw'))

    # load phantom pcd
    # recon_pcd, recon_mesh, recon_center = recon.load_mesh(params['recon_mesh'], recenter=True)
    target_pcd, recon_center = recon.load_reconpoints(params['recon_pcd'], recenter=True)
    # get center
    # recon_center = recon_pcd.get_center()

    # load camera positions
    camera_poses = recon.load_images(params['recon_images'])
    camera_poses = recon.calc_cam_center(camera_poses)
    camera_poses = recon.adjust_image_center(camera_poses, recon_center)
    camera_poses = recon.adjust_camera_scale(camera_poses, recon_center, scale=3.5)

    # calc focalpoint
    positions = recon.compute_focals(camera_poses)
    # positions,_= recon.flip_camera_x(positions)
    positions, _ = recon.flip_camera_y(positions)
    method = 'rotations' # 'rotations' or 'focalup'

    positions = sorted(positions, key=lambda x: x[3])
    count = 0
    pbar = tqdm(total=len(positions))
    for coord, focalpoint, cam_rotate, name in positions[::]:
    # for coord, focalpoint, cam_rotate, name in [positions[140]]:
    # for coord, focalpoint, cam_rotate, name in [positions[1050], positions[1100], positions[1600], positions[1849], positions[2049]]:
        if params['renderer'] == 'unity' and params['save']:
            # move camera here
            # calculate rotation
            # get dir
            # save view
            print(f'Coord: {coord}')
            print(f'Focal: {focalpoint}')
            print(f'Rotat: {cam_rotate}')
            # print(f'Provided Up: {cam_rotate-coord}')
            # cam_rotate[1] = -cam_rotate[1]
            print(name)
            if method == 'rotations':
                save_unity(name[:-4], coord, focalpoint, params['savedir'], params['modelname'], cam_rotate)
            else:
                save_unity(name[:-4], coord, focalpoint, params['savedir'], params['modelname'], up=cam_rotate)
            pass

        else:
            save_vtk(name[:-4], coord, focalpoint, params['savedir'], params['modelpath'], params['modelname'], params['save'],
                     cam_roll=cam_rotate[0])
        count += 1
        pbar.update(1)
        # print("Press Enter to continue...")
        input()
        # print("Continuing...")

    # dir2vid(params['savedir'], params['modelname'])

    return

def render_registered_pair_RtoP(params):
    if not os.path.isdir(os.path.join(params['savedir'], params['modelname'])):
        os.makedirs(os.path.join(params['savedir'], params['modelname']))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'render'))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'depth'))
        os.makedirs(os.path.join(params['savedir'], params['modelname'], 'raw'))

    # load phantom pcd
    # recon_pcd, recon_mesh, recon_center = recon.load_mesh(params['recon_mesh'], recenter=True)
    recon_pcd, recon_center = recon.load_reconpoints(params['recon_pcd'], recenter=True)
    # get center
    # recon_center = recon_pcd.get_center()

    # load camera positions
    camera_poses = recon.load_images(params['recon_images'])
    camera_poses = recon.calc_cam_center(camera_poses)
    camera_poses = recon.adjust_image_center(camera_poses, recon_center)
    camera_poses = recon.adjust_camera_scale(camera_poses, recon_center, scale=5)
    # print(camera_poses[0])
    version ='rigid'
    registered_poses = np.load(f'./data/soft3slowwet/registration3/phantom1_registered_cameras_{version}.npy')
    registered_focals = np.load(f'./data/soft3slowwet/registration3/phantom1_registered_focals_{version}.npy')
    camera_poses = recon.update_positions(camera_poses, registered_poses)
    # print(camera_poses[0])

    rotations = np.load(f'./data/soft3slowwet/registration3/phantom1_registered_rotations.npy')
    camera_poses = recon.update_rotations(camera_poses, rotations)
    # camera_poses = sorted(camera_poses, key=lambda x: x[4])

    # calc focalpoint
    positions = recon.compute_focals(camera_poses)

    registered_up = np.load(f'./data/soft3slowwet/registration3/phantom1_registered_up_{version}.npy')
    print(registered_up[55])
    print(registered_focals[55])
    method = 'focalup' # 'rotations' or 'focalup'
    if method == 'rotations':
        positions = recon.update_focals(positions, registered_focals)
        positions, registered_up = recon.flip_camera_y(positions)
    else:
        positions = recon.update_focals(positions, registered_focals, up=registered_up)
        positions, registered_up = recon.flip_camera_y(positions, up_positions=True)
    # positions = recon.update_focals(positions, registered_focals, up=registered_up)

    # print(f'Focal Norms: {recon.find_norms_focal(positions)}')
    # print(f'Up Norms: {recon.find_norms_up(positions)}')

    # positions, registered_up = recon.flip_camera_x(positions) # for unity
    # 461
    # positions, registered_up = recon.flip_camera_y(positions)
    # print(f'Focal Norms: {recon.find_norms_focal(positions)}')
    # print(f'Up Norms: {recon.find_norms_up(positions)}')
    positions = sorted(positions, key=lambda x: x[3])
    count = 0
    pbar = tqdm(total=len(positions))
    # for coord, focalpoint, cam_rotate, name in positions[100:101:]:
    # for coord, focalpoint, cam_rotate, name in positions[700:701:]:
    # for coord, focalpoint, cam_rotate, name in positions[1050:1051:]:
    # for coord, focalpoint, cam_rotate, name in positions[1195:1200:]:
    # for coord, focalpoint, cam_rotate, name in positions[1849:1850:]:
    # for coord, focalpoint, cam_rotate, name in positions[2049:2050:]:
    for coord, focalpoint, cam_rotate, name in [positions[1198]]:
    # for coord, focalpoint, cam_rotate, name in positions[::100]:
    # for coord, focalpoint, cam_rotate, name in [positions[1050], positions[1100], positions[1600], positions[1849], positions[2049]]:
        if params['renderer'] == 'unity' and params['save']:
            # move camera here
            # calculate rotation
            # get dir
            # save view
            # print(f'Coord: {coord}')
            # print(f'Focal: {focalpoint}')
            # print(f'Rotat: {cam_rotate}')
            # print(f'Provided Up: {cam_rotate-coord}')
            # print(f'DiffF:  {np.array(focalpoint) - np.array(coord)}')
            # print(f'DiffU:  {np.array(cam_rotate) - np.array(coord)}')
            # print(f'NormF:  {np.linalg.norm(np.array(focalpoint) - np.array(coord))}')
            # print(f'NormU:  {np.linalg.norm(np.array(cam_rotate) - np.array(coord))}')
            print(name)
            if method == 'rotations':
                save_unity(name[:-4], coord, focalpoint, params['savedir'], params['modelname'], cam_rotate)
            else:
                save_unity(name[:-4], coord, focalpoint, params['savedir'], params['modelname'], up=cam_rotate)
            pass

        else:
            save_vtk(name[:-4], coord, focalpoint, params['savedir'], params['modelpath'], params['modelname'], params['save'],
                     cam_roll=cam_rotate[0])
        count += 1
        pbar.update(1)
        # print("Press Enter to continue...")
        # input()
        # print("Continuing...")

    # dir2vid(params['savedir'], params['modelname'])

    return