import os
from tqdm import tqdm
from rendering import keypointPicker, render, labelVisualizer
from reachability import planner, reachability, analysis

params = {
        'cameradepth': 10.0,
        'fov': 87 // 2,  # cite boston scientific -- fiberoptic flexs 85
        'localbending': 25,
        'globalbending': 120,
        'wavesize': 10,
        'wavecount': 2,
        'visualize': False,
        'modelname': 'collectingsystem2',
        'models': {'collectingsystem2': (-41.4481495420699, 19.731541937774082, -97.36807944184557),
        'manualsegmentation1': (-45.75806986508405, 123.49623714056546, 1132.751798267298),
        'Patient1Right': (23.350248181136337, -137.49344143532295, 803.300038855812),
        'Patient3Left': (-52.889510869031284, -169.82580547336187, -442.6200299143518)}
    }

def main():

    modeldir = 'data/3dmodels'
    modelname = 'manualsegmentation1.stl'

    params['modelname'] = 'manualsegmentation1'
    collectingsystem, mask, lastcam = reachability.predict_reachability('data/3dmodels/manualsegmentation1.stl', params)


    reachability.render_reachable(collectingsystem, lastcam, mask)


def test():
    modeldir = 'data/3dmodels'
    modelname = 'Patient1Right.stl'
    keypointPicker.pickkeypoints(modeldir, modelname)

def paramsearch():
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    # labelVisualizer.visualize_labels(modelnames[1])

    wavesteps = [10,11,12,13,14,15]
    wavecount = [2,3,4,5]
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

                iou, dsc = analysis.analysis(modelnames[1], params)

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

def analyze():
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']

    totaliou = 0.0
    totaldsc = 0.0
    for name in modelnames:
        params['modelname'] = name
        iou, dsc = analysis.analysis(params)
        totaliou += iou
        totaldsc += dsc
        print(f'iou: {iou}')
        print(f'dsc: {dsc}')

    print(f'AVG IOU: {totaliou/len(modelnames)}')

    print(f'AVG DSC: {totaldsc/len(modelnames)}')

def proto():
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    params['modelname'] = modelnames[1]
    iou, dsc = analysis.analysis( params)
    print(f'iou: {iou}')
    print(f'dsc: {dsc}')

def label_viz():
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']

    labelVisualizer.visualize_labels(modelnames[3])

if __name__ == '__main__':
    # main()
    # paramsearch()
    # analyze()
    # proto()
    label_viz()

