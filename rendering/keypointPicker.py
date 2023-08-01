import os

# noinspection PyUnresolvedReferences
import vtkmodules.vtkInteractionStyle
# noinspection PyUnresolvedReferences
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkCommonCore import vtkIdTypeArray
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkFiltersExtraction import vtkExtractSelection
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkCommonDataModel import (
    vtkSelection,
    vtkSelectionNode,
    vtkUnstructuredGrid
)
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkLight,
    vtkCamera,
    vtkCellPicker,
    vtkDataSetMapper,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer,
    vtkWindowToImageFilter,
    vtkLightKit
)
from vtkmodules.vtkFiltersCore import (
    vtkSmoothPolyDataFilter,
    vtkPolyDataNormals
)
from vtkmodules.vtkFiltersSources import vtkSphereSource

from . import close_window

LOGNAME = ''


def rightclick_callback(caller, event):
    """
     Save keypoints and append to new document
    :param caller:
    :param event:
    :return:
    """
    # print(caller.GetClassName(), "clicked")
    # Print the interesting stuff.
    pos = caller.GetEventPosition()

    picker = vtkCellPicker()
    picker.SetTolerance(0.0005)
    renderer = caller.GetRenderWindow().GetRenderers().GetFirstRenderer()
    picker.Pick(pos[0], pos[1], 0, renderer)
    world_position = picker.GetPickPosition()
    print(f'Cell id is: {picker.GetCellId()}')

    if picker.GetCellId() != -1:
        print(f'Pick position is: ({world_position[0]:.6g}, {world_position[1]:.6g}, {world_position[2]:.6g})')
        coords = world_position


        global LOGNAME
        f = open(LOGNAME, 'a')
        f.write(f'{coords[0]}, {coords[1]}, {coords[2]}\n')
        f.close()

        # add a red dot
        sphere = vtkSphereSource()
        sphere.SetPhiResolution(10)
        sphere.SetThetaResolution(10)
        sphere.SetRadius(1.0)
        sphere.SetCenter(coords[0], coords[1], coords[2])
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtkActor()
        actor.SetMapper(mapper)
        colors = vtkNamedColors()
        actor.GetProperty().SetColor(colors.GetColor3d("Red"))
        renderer.AddActor(actor)
        caller.GetRenderWindow().Render()





def pickkeypoints(modeldir, modelname):
    global LOGNAME
    LOGNAME = os.path.join('output', modelname[:-4]+'.txt')
    f = open(LOGNAME, 'w')
    f.write('')
    f.close()

    colors = vtkNamedColors()
    colors.SetColor('HighNoonSun', [255, 255, 251, 255])  # Color temp. 5400°K
    colors.SetColor('100W Tungsten', [255, 214, 170, 255])

    # Create a reader
    reader = vtkSTLReader()
    reader.SetFileName(os.path.join(modeldir, modelname))

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
    # actor.GetProperty().SetDiffuse(0.8)
    # actor.GetProperty().SetDifffuseColor(colors.GetColor3d('LightSteelBlue'))
    actor.GetProperty().SetSpecular(0.1)
    actor.GetProperty().SetSpecularPower(2)
    actor.GetProperty().SetAmbient(0.0)
    actor.GetProperty().SetDiffuse(0.5)
    actor.GetProperty().SetColor(colors.GetColor3d('darksalmon'))
    actor.AddPosition(0,0,0)

    # Create a renderer, render window, and interactor
    renderer = vtkRenderer()
    # (array([  28.9986242 , -149.64840778,  815.07041743]), array([  28.15411734, -148.14308067,  813.91186293]))
    # 24.60988483418804 -138.2580149743589 802.4854295042734 24.805304367639536 -139.2312897081413 802.6630940348183
    # 43.62573020934702 -147.4097302127697 815.458252205618 43.384318110871895 -147.48930021207084 815.9727085659233
    x,y,z = 23.543271, -138.618929, 804.567765
    x2,y2,z2 = 28.278936, -148.365569, 814.083098
    #
    # camera = vtkCamera()
    # camera.SetPosition(x,y,z)
    # camera.SetFocalPoint(x2,y2,z2)
    # camera.SetDistance(1.771561)
    # renderer.SetActiveCamera(camera)

    renderer.RemoveAllLights()
    renderer.TwoSidedLightingOff()

    lighton = True
    if lighton:
        light1 = vtkLight()
        light1.SetPositional(True)
        light1.SetLightTypeToHeadlight()
        light1.SetPosition(x,y,z)
        light1.SetFocalPoint(x2,y2,z2)
        light1.SetColor(colors.GetColor3d('100W Tungsten'))
        light1.SetIntensity(4)
        renderer.AddLight(light1)

    # lighton = True
    # if lighton:
    #     light_kit = vtkLightKit()
    #     light_kit.SetKeyLightIntensity(1.0)
    #     light_kit.AddLightsToRenderer(renderer)

    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetWindowName('Camera')
    renderWindow.SetSize(512, 512)
    # renderWindow.Render()
    # renderWindow.GetActiveCamera().AddObserver('ModifiedEvent', camera_modified_callback)


    renderWindowInteractor = vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # Add the actor to the scene
    renderer.AddActor(actor)
    renderer.SetBackground(colors.GetColor3d('Black'))

    # Render and interact
    # renderWindowInteractor.Initialize()
    renderWindow.Render()    # renderer.GetActiveCamera().AddObserver('ModifiedEvent', camera_modified_callback)
    # renderWindowInteractor.Start()


    renderWindowInteractor.Initialize()
    renderWindow.Render()
    renderWindowInteractor.RemoveObservers('RightButtonPressEvent')
    renderWindowInteractor.AddObserver('RightButtonPressEvent', rightclick_callback)
    renderWindowInteractor.Start()

    # close_window(renderWindowInteractor)


if __name__ == '__main__':
    pickkeypoints()
