import os
from tqdm import tqdm
from rendering import keypointPicker, render, labelVisualizer, renderVideo
from reachability import planner, reachability, analysis
from multiprocessing import Pool
import copy


def main(params):
    '''
    DEPRECATED
    Parameters
    ----------
    params

    Returns
    -------

    '''

    modeldir = 'data/3dmodels'
    modelname = 'manualsegmentation1.stl'

    params['modelname'] = 'manualsegmentation1'
    collectingsystem, mask, lastcam = reachability.predict_reachability('data/3dmodels/manualsegmentation1.stl', params)


    reachability.render_reachable(collectingsystem, lastcam, mask)


def test(params):
    modeldir = 'data/3dmodels'
    modelname = 'Patient1Right.stl'
    keypointPicker.pickkeypoints(modeldir, modelname)

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
    avgdsc = totaldsc/len(modelnames)

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
    for step in range(1,10):
        params['wavesize'] = step
        for count in range(1,5):
            params['wavecount'] = count
            for depth in range(1,20):
                params['cameradepth'] = depth
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

def video_render(params):

    # saves a ton of images for visualization
    for name in ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']:
        params['modelname'] = name
        modelpath = os.path.join('data', '3dmodels', params['modelname'] + '.stl')
        renderVideo.render(modelpath, params)


if __name__ == '__main__':
    params = {
        'cameradepth': 14.0,
        'fov': 87 // 2,  # cite boston scientific -- fiberoptic flexs 85
        'localbending': 30,
        'globalbending': 150,
        'wavesize': 10,
        'wavecount': 2,
        'numpoints': 1, # how many camera points to sample
        'samplestep': 0.5,
        'visualize': False,
        'savedir': 'output_unfiltered',
        'save': False,
        'planmode': 'unfiltered', # bfs, dfs
        'modelname': 'Patient1Right',
        'models': {'collectingsystem2': (-41.4481495420699, 19.731541937774082, -97.36807944184557),
                   'manualsegmentation1': (-45.75806986508405, 123.49623714056546, 1132.751798267298),
                   'Patient1Right': (23.350248181136337, -137.49344143532295, 803.300038855812),
                   'Patient3Left': (-52.889510869031284, -169.82580547336187, -442.6200299143518)}
    }

    # main(params)
    # avgiou, avgdsc, _ = analyze(params)
    # print(f'AVG IOU: {avgiou}')
    # print(f'AVG DSC: {avgdsc}')
    # proto(params)
    # label_viz(params)
    # multiparamsearch(params)
    video_render(params)
