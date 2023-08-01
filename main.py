import os
from tqdm import tqdm
from rendering import keypointPicker, render, labelVisualizer
from reachability import planner, reachability



def main():
    # hyperparams
    fov = 87//2 # cite boston scientific
    # fiberoptic flexs 85
    localbending = 90
    globalbending = 120

    # import mesh
    # setup mesh mask
    modeldir = 'data/3dmodels'
    modelname = 'Patient1_New.stl'
    modelpath = os.path.join(modeldir, modelname)
    collectingsystem = render.load_mesh(modelpath)

    # generate camera positions
    positions = planner.gen_positions(modelpath, globalbending, localbending)
    mask = None
    lastcam = None
    count = 0
    pbar = tqdm(positions)
    for position in pbar:
        # if count>=1:break
        fakecam = reachability.gen_cam(position, fov)
        newmask = reachability.mark_intersection(collectingsystem, fakecam)
        mask = reachability.update_masks(mask, newmask)
        lastcam = fakecam
        count+=1

    reachability.render_reachable(collectingsystem, lastcam, mask)

def test():
    modeldir = 'data/3dmodels'
    modelname = 'Patient3Left.stl'
    keypointPicker.pickkeypoints(modeldir, modelname)

def vizlables():
    modelnames = ['collectingsystem2', 'manualsegmentation1', 'Patient1Right', 'Patient3Left']
    labelVisualizer.visualize_labels(modelnames[1])

if __name__ == '__main__':
    # main()
    vizlables()
