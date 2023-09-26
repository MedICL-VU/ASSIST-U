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

from rendering.render import load_meshactor, render

def create_label(coordinates, radius):
    sphere = vtkSphereSource()
    sphere.SetPhiResolution(10)
    sphere.SetThetaResolution(10)
    sphere.SetRadius(radius)
    sphere.SetCenter(coordinates[0], coordinates[1], coordinates[2])
    sphere.Update()
    return sphere.GetOutput()

def create_labelactor(coordinates, radius=1.0, color='Red'):
    mapper = vtkPolyDataMapper()
    mapper.SetInputDataObject(create_label(coordinates, radius))
    actor = vtkActor()
    actor.SetMapper(mapper)
    colors = vtkNamedColors()
    actor.GetProperty().SetColor(colors.GetColor3d(color))
    return actor

def load_labels(fname, radius=1.0):
    with open(fname, 'r') as f:
        coords=[]
        for line in f:

            coord = [float(x) for x in line.rstrip().split(', ')]
            coords.append(coord)
    contents = []
    for coord in coords:
        contents.append(create_label(coord, radius))
    return contents

def load_labelactors(fname, radius=1.0):
    with open(fname, 'r') as f:
        coords=[]
        for line in f:

            coord = [float(x) for x in line.rstrip().split(', ')]
            coords.append(coord)
    contents = []
    for coord in coords:
        contents.append(create_labelactor(coord, radius))
    return contents

def visualize_labels(modelname):
    modelpath = os.path.join('data', '3dmodels', modelname+'.stl')
    labelpath = os.path.join('data', 'labels', modelname + '.txt')
    model = load_meshactor(modelpath)
    labels = load_labelactors(labelpath, 1.0)
    render([model]+ labels)


if __name__ == "__main__":
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    visualize_labels(modelnames[2])