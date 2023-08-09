import os
from tqdm import tqdm
from rendering import keypointPicker, render, labelVisualizer
from reachability import planner, reachability, analysis
from multiprocessing import Pool
import copy


def main(params):
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    params['modelname'] = 'manualsegmentation1'
    collectingsystem, mask, lastcam = reachability.predict_reachability('data/3dmodels/manualsegmentation1.stl', params)

    reachability.render_reachable(collectingsystem, lastcam, mask)


def render_skeleton_camera(params):
    params['modelname'] = 'Patient1Right'
    collectingsystem, mask, lastcam, positions = reachability.predict_reachability(
        'data/3dmodels/Patient1Right.stl', params)
    pointactors = []
    for position in positions:

        coord, focal = position
    # coord = coord - ((0.5 * params['cameradepth']) * focal)
    # focal = -1* focal
        pointactors.append(labelVisualizer.create_labelactor(coord, 0.25, 'Red'))
    # pointactors.append(labelVisualizer.create_labelactor(coord+(focal*params['cameradepth']), 1, 'Blue'))

    render.render2(collectingsystem, lastcam, mask, pointactors)

def render_points_camera(params):
    params['modelname'] = 'Patient1Right'
    collectingsystem, mask, lastcam, position = reachability.predict_reachability(
        'data/3dmodels/Patient1Right.stl', params)
    pointactors = []

    coord, focal = position
    # coord = coord - ((0.5 * params['cameradepth']) * focal)
    # focal = -1* focal
    pointactors.append(labelVisualizer.create_labelactor(coord, 1, 'Blue'))
    # pointactors.append(labelVisualizer.create_labelactor(coord+(focal*params['cameradepth']), 1, 'Blue'))


    render.render2(collectingsystem, lastcam, mask, pointactors)

def test(params):
    modeldir = 'data/3dmodels'
    modelname = 'Patient3Left.stl'
    keypointPicker.pickkeypoints(modeldir, modelname)

def analyze(params):
    # modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    modelnames = ['Patient1Right']
    totaliou = 0.0
    totaldsc = 0.0
    for name in modelnames:
        params['modelname'] = name
        iou, dsc = analysis.analysis(params)
        totaliou += iou
        totaldsc += dsc
        # print(f'iou: {iou}')
        # print(f'dsc: {dsc}')
    avgiou = totaliou/len(modelnames)
    avgdsc =totaldsc/len(modelnames)

    # print(f'AVG IOU: {avgiou}')
    # print(f'AVG DSC: {avgdsc}')
    return avgiou, avgdsc, params

def proto(params):
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    params['modelname'] = modelnames[2]
    iou, dsc = analysis.analysis( params)
    print(f'iou: {iou}')
    print(f'dsc: {dsc}')

def label_viz(params):
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']

    labelVisualizer.visualize_labels(modelnames[1])


def multiparamsearch(params):
    cpucount = os.cpu_count()
    print(f'Available processes {cpucount}')
    print(f'Using {cpucount -1}')


    # construct params
    wavesteps = [5,6,7,8,9,10]
    wavecount = [1,2,3]

    work = []

    # for step in wavesteps:
    #     params['wavesize'] = step
    #     for count in wavecount:
    #         params['wavecount'] = count
    #         work.append(copy.deepcopy(params))
    for step in range(1,15):
        params['wavesize'] = step
        for count in range(1,5):
            params['wavecount'] = count
            # for depth in range(1,20):
            #     params['cameradepth'] = depth
            work.append(copy.deepcopy(params))
    p = Pool(cpucount - 1)
    results = p.imap(analyze, work)

    # search results
    bestiou = 0
    iouparam = None

    bestdsc = 0
    dscparam = None
    for iou, dsc, param in results:
        if iou > bestiou:
            bestiou = iou
            iouparam = param
        if dsc > bestdsc:
            bestdsc = dsc
            dscparam = param

    # search results

    print(f'BESTIOU: {bestiou}')
    print(f'Param: {iouparam}')

    print(f'BESTDSC: {bestdsc}')
    print(f'Param: {dscparam}')

if __name__ == '__main__':
    params = {
        'cameradepth': 7.0,
        'fov': 87 // 2,  # boston scientific -- digital ==87 -- fiberoptic flexs== 85
        'localbending': 90, # will be a higher number for fiber optic
        'globalbending': 150,
        'wavesize': 5,
        'wavecount': 1,
        'visualize': False,
        'modelname': 'collectingsystem2',
        'models': {'collectingsystem2': (-41.0082, 19.8301, -96.8928),
                   'manualsegmentation1': (-45.649, 124.412, 1132.38),
                   'Patient1Right': (24.0183, -135.715, 803.426),
                   'Patient3Left': (-51.8847, -169.306, -443.374)}
    }

    # main(params)
    # render_points_camera(params)
    render_skeleton_camera(params)
    # print(analyze(params))
    # proto(params)
    # label_viz(params)
    # multiparamsearch(params)

    # test(params)