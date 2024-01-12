import os
import math
import numpy as np
from PIL import Image
from rendering import render
from reachability import planner

# noinspection PyUnresolvedReferences
import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter

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

def stitch_dmap(rgb_outputs, depth_outputs, x,y):
    dmap = np.zeros((x,y))
    for rgb, depth in zip(rgb_outputs, depth_outputs):
        # where rgb != 255,255,255
        # quick_sum = np.sum(rgb, axis=2)
        # image_mask = quick_sum != 255*3
        image_mask = (rgb != [0,255,0]).all(axis=2)
        dmap_mask = dmap == 0
        # where dmap still 0, update with latest depth where rgb is positive
        dmap = np.where(np.logical_and(image_mask, dmap_mask),depth, dmap)

    return dmap

def save_view(id, coord, focal, savedir, modelpath, modelname, save):
    # string, tuple, tuple
    colors = vtkNamedColors()
    colors.SetColor('100W Tungsten', [255, 214, 170, 255])
    colors.SetColor('Full Green', [0,255,0,255])

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
    actor.GetProperty().SetSpecular(0.1)
    actor.GetProperty().SetSpecularPower(2)
    actor.GetProperty().SetAmbient(0)
    actor.GetProperty().SetDiffuse(5)
    actor.GetProperty().SetColor(colors.GetColor3d('darksalmon'))

    camera = vtkCamera()
    camera.SetPosition(coord[0], coord[1], coord[2])
    camera.SetFocalPoint(focal[0], focal[1], focal[2])

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
    light1.SetAttenuationValues(0.5,1,0.1)
    # print(light1.GetConeAngle())
    # light1.SetConeAngle(30)
    # light1.SetColor(colors.GetColor3d('White'))
    light1.SetIntensity(4)
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
    renderer.SetBackground(colors.GetColor3d('Full Green'))
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
        writer.SetFileName(os.path.join(savedir, modelname, f'{id}_render.png'))
        writer.SetInputConnection(w2if.GetOutputPort())
        writer.Write()

        # depth

        # w2if.SetInput(renderWindow)
        # w2if.SetInputBufferTypeToZBuffer()
        # # w2if.ReadFrontBufferOn()
        # w2if.Update()
        # converter = vtkImageShiftScale()
        # converter.SetOutputScalarTypeToUnsignedShort()
        # converter.SetInputData(w2if.GetOutput())
        # # converter.SetClampOverflow(True)
        # # converter.SetShift(0)
        # converter.SetScale(255)
        # converter.Update()
        #use displaytoworld instead?

        # writer = vtkPNGWriter()
        # writer.SetFileName(os.path.join(savedir, modelname, f'{id}_depth.png'))
        # writer.SetInputConnection(converter.GetOutputPort())
        # writer.Write()

        # manual depth - picking
        # dmap = np.zeros((x,y))
        # https://discourse.vtk.org/t/deriving-coordinates-for-display-pixel-values/9364 how to vectorize in future

        # for i in range(x):
        #     for j in range(y):
        #         picker = vtkPropPicker()
        #         picker.Pick(i, j, 0, renderer)
        #         pos = picker.GetPickPosition()
        #         # worldcoord = renderer.DisplayToWorld(vtkVector3d(i,j,0))
        #         dmap[i][j] = math.sqrt(vtkMath.Distance2BetweenPoints(pos, coord))
        #         # dmap[i][j]
        #
        # dmap = 255.0 *(dmap - dmap.min())/dmap.ptp()
        # im = Image.fromarray(np.rot90(dmap))
        # im = im.convert('RGB')
        # im.save(os.path.join(savedir, modelname, f'{id}_manualdepth.png'))

        # manual depth 2 - stitching
        # ratios = [(0.1,1), (1,5), (5,10), (10,50),(50,100), (100,500), (500,1000)]
        ratios = [(1e-5,0.1),
                  (0.1,0.5),
                  (0.5,1),
                  (1,2),
                  (2,5),
                  (5,10),
                  (10,20),
                  (20,50),
                  (50,100),
                  (100,500),
                  (500,1000),]
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
        im.save(os.path.join(savedir, modelname, f'{id}_dmap_raw.tiff'))

        dmap = 255.0 * (dmap - dmap.min()) / dmap.ptp()
        im = Image.fromarray(dmap)
        im = im.convert('L')
        im.save(os.path.join(savedir, modelname, f'{id}_dmap.png'))


        # print(camera.GetParallelProjection())

        # renderWindow.Finalize()
        with open(os.path.join(savedir, modelname)+'.txt', 'a') as file:
            file.write(f'{id}_render.png {" ".join([str(num) for num in coord])} {" ".join([str(num) for num in focal])} {" ".join([str(num) for num in camera.GetOrientationWXYZ()])}\n')
    else:
        renderWindowInteractor = vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        renderWindowInteractor.Initialize()
        renderWindowInteractor.Start()

def render(modelpath, params):
    if not os.path.isdir(os.path.join(params['savedir'], params['modelname'])):
        os.makedirs(os.path.join(params['savedir'], params['modelname']))

    # generate camera positions
    positions = planner.gen_positions(modelpath, params)

    count = 0
    # pbar = tqdm(positions)
    for coord, focalpoint in positions:
        if count>=1:break
        focalpoint = coord + focalpoint
        save_view(f'{count:05}', coord, focalpoint, params['savedir'], modelpath, params['modelname'], params['save'])
        count += 1


    return