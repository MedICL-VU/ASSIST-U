import os
from tqdm import tqdm
from rendering import keypointPicker, render, labelVisualizer
from reachability import planner, reachability, analysis
from multiprocessing import Pool
import copy


def main(params):

    modeldir = 'data/3dmodels'
    modelname = 'manualsegmentation1.stl'

    params['modelname'] = 'manualsegmentation1'
    collectingsystem, mask, lastcam = reachability.predict_reachability('data/3dmodels/manualsegmentation1.stl', params)


    reachability.render_reachable(collectingsystem, lastcam, mask)


def test(params):
    modeldir = 'data/3dmodels'
    modelname = 'Patient1Right.stl'
    keypointPicker.pickkeypoints(modeldir, modelname)

def paramsearch(params):
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    # labelVisualizer.visualize_labels(modelnames[1])

    wavesteps = [10,11,12,13,14]
    wavecount = [1,2,3]
    bestiou = 0
    ioustep = 0
    ioucount = 0

    bestdsc = 0
    dscstep = 0
    dsccount = 0
    with tqdm(total=len(wavesteps)*len(wavecount)) as pbar:
        for step in wavesteps:
            params['wavesize'] = step
            for count in wavecount:
                params['wavecount'] = count

                # iou, dsc = analysis.analysis(modelnames[1], params)
                iou, dsc = analyze()

                if iou > bestiou:
                    bestiou = iou
                    ioustep = step
                    ioucount = count
                if dsc > bestdsc:
                    bestdsc = dsc
                    dscstep = step
                    dsccount = count
                pbar.update(1)
    print(f'BESTIOU: {bestiou}')
    print(f'Step size: {ioustep}')
    print(f'Count: {ioucount}')
    print(f'BESTDSC: {bestdsc}')
    print(f'Step size: {dscstep}')
    print(f'Count: {dsccount}')

def analyze(params):
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']

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

    p = Pool(cpucount - 1)

    # construct params
    wavesteps = [5,6,7,8,9,10]
    wavecount = [1,2,3]

    work = []

    for step in wavesteps:
        params['wavesize'] = step
        for count in wavecount:
            params['wavecount'] = count
            work.append(copy.deepcopy(params))

    results = p.map(analyze, work)

    # search results
    bestiou = 0
    ioustep = 0
    ioucount = 0

    bestdsc = 0
    dscstep = 0
    dsccount = 0
    for iou, dsc, param in results:
        if iou > bestiou:
            bestiou = iou
            ioustep = param['wavesize']
            ioucount = param['wavecount']
        if dsc > bestdsc:
            bestdsc = dsc
            dscstep = param['wavesize']
            dsccount = param['wavecount']

    # search results

    print(f'BESTIOU: {bestiou}')
    print(f'Step size: {ioustep}')
    print(f'Count: {ioucount}')
    print(f'BESTDSC: {bestdsc}')
    print(f'Step size: {dscstep}')
    print(f'Count: {dsccount}')

if __name__ == '__main__':
    params = {
        'cameradepth': 10.0,
        'fov': 87 // 2,  # cite boston scientific -- fiberoptic flexs 85
        'localbending': 25,
        'globalbending': 120,
        'wavesize': 8,
        'wavecount': 1,
        'visualize': True,
        'modelname': 'collectingsystem2',
        'models': {'collectingsystem2': (-41.4481495420699, 19.731541937774082, -97.36807944184557),
                   'manualsegmentation1': (-45.75806986508405, 123.49623714056546, 1132.751798267298),
                   'Patient1Right': (23.350248181136337, -137.49344143532295, 803.300038855812),
                   'Patient3Left': (-52.889510869031284, -169.82580547336187, -442.6200299143518)}
    }

    # main(params)
    # paramsearch(params)
    # analyze(params)
    # proto(params)
    # label_viz(params)
    # multiparamsearch(params)
    modelnames = ['collectingsystem2.stl', 'manualsegmentation1.stl', 'Patient1Right.stl', 'Patient3Left.stl']
    keypointPicker.pickkeypoints('data/3dmodels',modelnames[3])
