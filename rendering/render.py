import numpy as np
import os
import glob
import cv2
import copy

import open3d as o3d
import numpy as np
import scipy
import skeletor as sk
import trimesh

# noinspection PyUnresolvedReferences
import vtkmodules.vtkInteractionStyle
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkIOImage import vtkPNGWriter

from vtkmodules.vtkIOGeometry import (
    vtkBYUReader,
    vtkOBJReader,
    vtkSTLReader
)
from vtkmodules.vtkIOLegacy import vtkPolyDataReader
from vtkmodules.vtkIOPLY import vtkPLYReader
from vtkmodules.vtkIOXML import vtkXMLPolyDataReader

from vtkmodules.vtkFiltersSources import vtkConeSource

from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkLight,
    vtkCamera,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer,
    vtkWindowToImageFilter,
    vtkDataSetMapper
)
from vtkmodules.vtkFiltersCore import (
    vtkSmoothPolyDataFilter,
    vtkPolyDataNormals,
    vtkCenterOfMass,
    vtkArrayCalculator,
    vtkAppendPolyData,
    vtkCleanPolyData

)
from vtkmodules.vtkFiltersGeneral import (
    vtkMultiThreshold,
    vtkTransformPolyDataFilter,
    vtkBooleanOperationPolyDataFilter,
    vtkMergeArrays
)
from vtkmodules.vtkFiltersSources import vtkSphereSource
from vtkmodules.vtkFiltersModeling import vtkSelectEnclosedPoints
from vtkmodules.vtkCommonDataModel import vtkDataObject
from vtk.util.numpy_support import vtk_to_numpy, numpy_to_vtk
from tqdm import tqdm
from rendering import keypointPicker
from reachability import planner, reachability

def ReadPolyData(file_name):
    path, extension = os.path.splitext(file_name)
    extension = extension.lower()
    if extension == '.ply':
        reader = vtkPLYReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == '.vtp':
        reader = vtkXMLPolyDataReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == '.obj':
        reader = vtkOBJReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == '.stl':
        reader = vtkSTLReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == '.vtk':
        reader = vtkPolyDataReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == '.g':
        reader = vtkBYUReader()
        reader.SetGeometryFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    else:
        # Return a None if the extension is unknown.
        poly_data = None
    return poly_data

def load_mesh(modelpath):

    # Create a reader
    model = ReadPolyData(modelpath)

    smoothFilter = vtkSmoothPolyDataFilter()
    smoothFilter.SetInputDataObject(model)
    smoothFilter.SetNumberOfIterations(15)
    smoothFilter.SetRelaxationFactor(0.1)
    smoothFilter.FeatureEdgeSmoothingOff()

    smoothFilter.BoundarySmoothingOn()
    smoothFilter.Update()

    return smoothFilter.GetOutput()

def load_meshactor(modelpath):
    colors = vtkNamedColors()
    pdata = load_mesh(modelpath)
    # Create a mapper and actor
    mapper = vtkPolyDataMapper()
    # mapper.SetInputConnection(reader.GetOutputPort())
    mapper.SetInputDataObject(pdata)
    actor = vtkActor()
    actor.SetMapper(mapper)
    # actor.GetProperty().SetDiffuse(0.8)
    # actor.GetProperty().SetDifffuseColor(colors.GetColor3d('LightSteelBlue'))
    actor.GetProperty().SetSpecular(0.1)
    actor.GetProperty().SetSpecularPower(2)
    actor.GetProperty().SetAmbient(0.0)
    actor.GetProperty().SetDiffuse(0.5)
    actor.GetProperty().SetColor(colors.GetColor3d('darksalmon'))
    actor.AddPosition(0, 0, 0)
    return actor


def render(actors):
    colors = vtkNamedColors()
    colors.SetColor('100W Tungsten', [255, 214, 170, 255])

    renderer = vtkRenderer()
    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetWindowName('Camera')
    renderWindow.SetSize(512, 512)


    renderWindowInteractor = vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    x, y, z = 23.543271, -138.618929, 804.567765
    x2, y2, z2 = 28.278936, -148.365569, 814.083098

    renderer.RemoveAllLights()
    renderer.TwoSidedLightingOff()

    lighton = True
    if lighton:
        light1 = vtkLight()
        light1.SetPositional(True)
        light1.SetLightTypeToHeadlight()
        light1.SetPosition(x, y, z)
        light1.SetFocalPoint(x2, y2, z2)
        light1.SetColor(colors.GetColor3d('100W Tungsten'))
        light1.SetIntensity(4)
        renderer.AddLight(light1)

    # Add the actors to the scene
    for actor in actors:
        renderer.AddActor(actor)

    renderer.SetBackground(colors.GetColor3d('Black'))

    # Render and interact
    renderWindow.Render()
    renderWindowInteractor.Initialize()
    renderWindowInteractor.Start()

def render2(polyData1, polyData2, mask, actors):
    # Extract three meshes, one completely inside, one completely
    # outside and on the border between the inside and outside.

    threshold = vtkMultiThreshold()
    # Outside points have a 0 value in ALL points of a cell
    outsideId = threshold.AddBandpassIntervalSet(
        0, 0,
        vtkDataObject.FIELD_ASSOCIATION_POINTS, 'SelectedPoints',
        0, 1)
    # Inside points have a 1 value in ALL points of a cell
    insideId = threshold.AddBandpassIntervalSet(
        1, 1,
        vtkDataObject.FIELD_ASSOCIATION_POINTS, 'SelectedPoints',
        0, 1)
    # Border points have a 0 or a 1 in at least one point of a cell
    borderId = threshold.AddIntervalSet(
        0, 1,
        vtkMultiThreshold.OPEN, vtkMultiThreshold.OPEN,
        vtkDataObject.FIELD_ASSOCIATION_POINTS, 'SelectedPoints',
        0, 0)

    # threshold.SetInputConnection(mask.GetOutputPort())

    threshold.SetInputData(mask)

    # Select the intervals to be output
    threshold.OutputSet(outsideId)
    threshold.OutputSet(insideId)
    threshold.OutputSet(borderId)
    threshold.Update()

    # Visualize
    colors = vtkNamedColors()
    outsideColor = colors.GetColor3d('Grey')
    insideColor = colors.GetColor3d('Grey')
    borderColor = colors.GetColor3d('Grey')
    surfaceColor = colors.GetColor3d('Peacock')
    backgroundColor = colors.GetColor3d('Silver')

    # Outside
    outsideMapper = vtkDataSetMapper()
    outsideMapper.SetInputData(threshold.GetOutput().GetBlock(outsideId).GetBlock(0))
    outsideMapper.ScalarVisibilityOff()

    outsideActor = vtkActor()
    outsideActor.SetMapper(outsideMapper)
    outsideActor.GetProperty().SetDiffuseColor(outsideColor)
    outsideActor.GetProperty().SetOpacity(.5)
    outsideActor.GetProperty().SetSpecular(.6)
    outsideActor.GetProperty().SetSpecularPower(30)
    # outsideActor.GetProperty().SetRepresentationToWireframe()

    # Inside
    insideMapper = vtkDataSetMapper()
    insideMapper.SetInputData(threshold.GetOutput().GetBlock(insideId).GetBlock(0))
    insideMapper.ScalarVisibilityOff()

    insideActor = vtkActor()
    insideActor.SetMapper(insideMapper)
    insideActor.GetProperty().SetDiffuseColor(insideColor)
    insideActor.GetProperty().SetOpacity(.5)
    insideActor.GetProperty().SetSpecular(.6)
    insideActor.GetProperty().SetSpecularPower(30)
    # insideActor.GetProperty().EdgeVisibilityOn()
    # insideActor.GetProperty().SetRepresentationToWireframe()

    # Border
    borderMapper = vtkDataSetMapper()
    borderMapper.SetInputData(threshold.GetOutput().GetBlock(borderId).GetBlock(0))
    borderMapper.ScalarVisibilityOff()

    borderActor = vtkActor()
    borderActor.SetMapper(borderMapper)
    borderActor.GetProperty().SetDiffuseColor(borderColor)
    borderActor.GetProperty().SetOpacity(.5)
    borderActor.GetProperty().SetSpecular(.6)
    borderActor.GetProperty().SetSpecularPower(30)
    # borderActor.GetProperty().EdgeVisibilityOn()
    # borderActor.GetProperty().SetRepresentationToWireframe()

    surfaceMapper = vtkDataSetMapper()
    surfaceMapper.SetInputData(polyData2)  # HERERERERE
    surfaceMapper.ScalarVisibilityOff()

    # Surface of object containing cell
    surfaceActor = vtkActor()
    surfaceActor.SetMapper(surfaceMapper)
    surfaceActor.GetProperty().SetDiffuseColor(surfaceColor)
    surfaceActor.GetProperty().SetOpacity(.5)
    # surfaceActor.GetProperty().EdgeVisibilityOn()

    renderer = vtkRenderer()
    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(640, 480)

    renderWindowInteractor = vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    renderer.SetBackground(backgroundColor)
    renderer.UseHiddenLineRemovalOn()

    renderer.AddActor(surfaceActor)
    renderer.AddActor(outsideActor)
    renderer.AddActor(insideActor)
    renderer.AddActor(borderActor)
    for actor in actors:
        renderer.AddActor(actor)

    renderWindow.SetWindowName('CellsInsideObject')
    renderWindow.Render()
    renderer.GetActiveCamera().Azimuth(30)
    renderer.GetActiveCamera().Elevation(30)
    renderer.GetActiveCamera().Dolly(1.25)
    renderWindow.Render()

    renderWindowInteractor.Start()
    return