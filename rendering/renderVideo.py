import os
import math
import numpy as np
from PIL import Image
from rendering import render
from reachability import planner
from reachability import reachability

# noinspection PyUnresolvedReferences
import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from util.dirtovid import dir2vid

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

def save_unity(id, coord, focal, savedir, modelname):

    # currently assumes model already loaded
    send_command(f'MoveCamera,{-coord[0]},{coord[1]},{coord[2]}')
    send_command(f'RotateCameraLook,{-focal[0]},{focal[1]},{focal[2]}')

    fpath = os.path.join(os.getcwd(), savedir, modelname)
    rendername= os.path.join(fpath,'render', id+'_render.png')
    send_command(f'Screenshot,{rendername},Normal')
    depthname = os.path.join(fpath, 'depth', id + '_dmap.png')
    send_command(f'Screenshot,{depthname},Depth')

def save_vtk(id, coord, focal, savedir, modelpath, modelname, save):
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
                  (100,250)]
                  # (100,500),
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
            save_unity(f'{count:05}', coord, focalpoint, params['savedir'], params['modelname'])
        else:
            save_vtk(f'{count:05}', coord, focalpoint, params['savedir'], params['modelpath'], params['modelname'], params['save'])
        count += 1

    # dir2vid(params['savedir'], params['modelname'])

    return