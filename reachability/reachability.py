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
from vtkmodules.vtkFiltersModeling import vtkSelectEnclosedPoints
from vtkmodules.vtkCommonDataModel import vtkDataObject
from vtk.util.numpy_support import vtk_to_numpy, numpy_to_vtk
from tqdm import tqdm
from rendering import keypointPicker
from reachability import planner


def gen_cam(position, fov):
    coord, focal = position
    cone = vtkConeSource()
    cone.SetHeight(10.0)
    # cone.SetRadius(3.0)
    cone.SetAngle(fov)
    cone.SetResolution(10)
    cone.SetCenter(coord[0], coord[1], coord[2])
    cone.SetDirection(focal[0], focal[1], focal[2])
    cone.Update()

    # set position
    # transform = vtkTransform()
    # transform.Translate(coord[0], coord[1], coord[2])
    #
    # transformPD = vtkTransformPolyDataFilter()
    # transformPD.SetTransform(transform)
    # transformPD.SetInputData(cone.GetOutput())
    # transformPD.Update()
    # conedata = transformPD.GetOutput()
    return cone.GetOutput()

def boolean_polydata(mask1, mask2, operation):
    booleanOperation = vtkBooleanOperationPolyDataFilter()
    if operation.lower() == 'union':
        booleanOperation.SetOperationToUnion()
    elif operation.lower() == 'intersection':
        booleanOperation.SetOperationToIntersection()
    elif operation.lower() == 'difference':
        booleanOperation.SetOperationToDifference()
    else:
        print('Unknown operation:', operation)
        return
    booleanOperation.SetInputData(0, mask1)
    booleanOperation.SetInputData(1, mask2)
    booleanOperation.Update()
    return booleanOperation.GetOutput()

def update_masks(mask1, mask2):
    if mask1 is None:
        return mask2
    # extract array and union them?
    # array1 = mask1.GetPoints().GetData()
    # array2 = mask2.GetPoints().GetData()
    # merger = vtkMergeArrays()
    #
    # calc = vtkArrayCalculator()

    # this doensn't work
    # appendFilter = vtkAppendPolyData()
    # appendFilter.AddInputData(mask1)
    # appendFilter.AddInputData(mask2)
    # appendFilter.Update()
    # cleanFilter = vtkCleanPolyData()
    # cleanFilter.SetInputConnection(appendFilter.GetOutputPort())
    # cleanFilter.Update()
    array1 = vtk_to_numpy(mask1.GetPointData().GetAbstractArray('SelectedPoints'))
    array2 = vtk_to_numpy(mask2.GetPointData().GetAbstractArray('SelectedPoints'))
    for i in range(len(array1)):
        array1[i] = array1[i] | array2[i]
    # array1 = np.logical_or(array1,array2)
    # mask1.GetPointData().SetAbstractArray('SelectedPoints', array3)

    return mask1

    # return mask1

def mark_intersection(polyData1, polyData2):
    # from https://examples.vtk.org/site/Python/PolyData/CellsInsideObject/
    # Mark points inside with 1 and outside with a 0
    select = vtkSelectEnclosedPoints()
    select.SetInputData(polyData1)
    select.SetSurfaceData(polyData2)

    select.Update()
    return select.GetOutput()

def render_reachable(polyData1, polyData2, mask):
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
    outsideColor = colors.GetColor3d('Crimson')
    insideColor = colors.GetColor3d('Banana')
    borderColor = colors.GetColor3d('Mint')
    surfaceColor = colors.GetColor3d('Peacock')
    backgroundColor = colors.GetColor3d('Silver')

    # Outside
    outsideMapper = vtkDataSetMapper()
    outsideMapper.SetInputData(threshold.GetOutput().GetBlock(outsideId).GetBlock(0))
    outsideMapper.ScalarVisibilityOff()

    outsideActor = vtkActor()
    outsideActor.SetMapper(outsideMapper)
    outsideActor.GetProperty().SetDiffuseColor(outsideColor)
    outsideActor.GetProperty().SetSpecular(.6)
    outsideActor.GetProperty().SetSpecularPower(30)

    # Inside
    insideMapper = vtkDataSetMapper()
    insideMapper.SetInputData(threshold.GetOutput().GetBlock(insideId).GetBlock(0))
    insideMapper.ScalarVisibilityOff()

    insideActor = vtkActor()
    insideActor.SetMapper(insideMapper)
    insideActor.GetProperty().SetDiffuseColor(insideColor)
    insideActor.GetProperty().SetSpecular(.6)
    insideActor.GetProperty().SetSpecularPower(30)
    insideActor.GetProperty().EdgeVisibilityOn()

    # Border
    borderMapper = vtkDataSetMapper()
    borderMapper.SetInputData(threshold.GetOutput().GetBlock(borderId).GetBlock(0))
    borderMapper.ScalarVisibilityOff()

    borderActor = vtkActor()
    borderActor.SetMapper(borderMapper)
    borderActor.GetProperty().SetDiffuseColor(borderColor)
    borderActor.GetProperty().SetSpecular(.6)
    borderActor.GetProperty().SetSpecularPower(30)
    borderActor.GetProperty().EdgeVisibilityOn()

    surfaceMapper = vtkDataSetMapper()
    surfaceMapper.SetInputData(polyData2) #HERERERERE
    surfaceMapper.ScalarVisibilityOff()

    # Surface of object containing cell
    surfaceActor = vtkActor()
    surfaceActor.SetMapper(surfaceMapper)
    surfaceActor.GetProperty().SetDiffuseColor(surfaceColor)
    surfaceActor.GetProperty().SetOpacity(.1)

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

    renderWindow.SetWindowName('CellsInsideObject')
    renderWindow.Render()
    renderer.GetActiveCamera().Azimuth(30)
    renderer.GetActiveCamera().Elevation(30)
    renderer.GetActiveCamera().Dolly(1.25)
    renderWindow.Render()

    renderWindowInteractor.Start()
    return [], [], []