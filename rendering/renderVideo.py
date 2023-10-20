import os
from rendering import render
from reachability import planner

# noinspection PyUnresolvedReferences
import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkLight,
    vtkCamera,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer,
    vtkWindowToImageFilter
)

from vtkmodules.vtkFiltersCore import (
    vtkSmoothPolyDataFilter,
    vtkPolyDataNormals
)

def save_view(id, coord, focal, savedir, modelpath, modelname, save):
    # string, tuple, tuple
    colors = vtkNamedColors()
    colors.SetColor('100W Tungsten', [255, 214, 170, 255])

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
    actor.GetProperty().SetSpecularPower(0.5)
    actor.GetProperty().SetColor(colors.GetColor3d('darksalmon'))

    camera = vtkCamera()
    camera.SetPosition(coord[0], coord[1], coord[2])
    camera.SetFocalPoint(focal[0], focal[1], focal[2])

    # Create a renderer, render window, and interactor
    renderer = vtkRenderer()
    renderer.SetActiveCamera(camera)

    renderer.RemoveAllLights()
    renderer.TwoSidedLightingOff()

    lighton = True
    if lighton:
        light1 = vtkLight()
        light1.SetPositional(True)
        light1.SetLightTypeToHeadlight()
        light1.SetPosition(coord[0], coord[1], coord[2])
        light1.SetFocalPoint(focal[0], focal[1], focal[2])
        light1.SetColor(colors.GetColor3d('100W Tungsten'))
        light1.SetIntensity(3)
        renderer.AddLight(light1)

    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetWindowName('Camera')
    renderWindow.SetSize(512, 512)
    renderWindow.Render()


    # Add the actor to the scene
    renderer.AddActor(actor)
    renderer.SetBackground(colors.GetColor3d('MistyRose'))

    if save:
        w2if = vtkWindowToImageFilter()
        w2if.SetInput(renderWindow)
        w2if.SetInputBufferTypeToRGB()
        w2if.ReadFrontBufferOff()
        w2if.Update()
        writer = vtkPNGWriter()
        writer.SetFileName(os.path.join(savedir, modelname, f'{id}_render.png'))
        writer.SetInputConnection(w2if.GetOutputPort())
        writer.Write()

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
        # if count>=1:break
        focalpoint = coord + focalpoint
        save_view(f'{count:05}', coord, focalpoint, params['savedir'], modelpath, params['modelname'], params['save'])
        count += 1

    return