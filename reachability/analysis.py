import os
from reachability import reachability
from tqdm import tqdm
from rendering import render, labelVisualizer
from vtk.util.numpy_support import vtk_to_numpy
from util import metrics

def invertmask(mask):

    array1 = vtk_to_numpy(mask.GetPointData().GetAbstractArray('SelectedPoints'))

    for i in range(len(array1)):
        if array1[i] == 1:
            array1[i] = 0
        else:
            array1[i] = 1
    return mask

def gen_labeledmodel(model, labels):
    # polydata objects
    mask = None
    for label in labels:
        # if count>=1:break
        newmask = reachability.mark_intersection(model, label)
        mask = reachability.update_masks(mask, newmask)
    mask = invertmask(mask)
    return mask


def compare(mask1, mask2):
    array1 = vtk_to_numpy(mask1.GetPointData().GetAbstractArray('SelectedPoints'))
    array2 = vtk_to_numpy(mask2.GetPointData().GetAbstractArray('SelectedPoints'))
    iou = metrics.iou(array1, array2)
    dsc = metrics.dice(array1, array2)
    return iou, dsc

def analysis(params):
    # expects polydata objects
    modelpath = os.path.join('data', '3dmodels', params['modelname'] + '.stl')
    labelpath = os.path.join('data', 'labels', params['modelname'] + '.txt')
    model = render.load_mesh(modelpath)
    labels = labelVisualizer.load_labels(labelpath, 2.0)
    labelmask = gen_labeledmodel(model, labels)
    _, predictmask, lastcam = reachability.predict_reachability(modelpath, params)
    iou, dsc = compare(labelmask, predictmask)
    if params['visualize']:
        reachability.render_reachable(model, labelmask, labelmask)
        reachability.render_reachable(model, labelmask, predictmask)
    return iou, dsc