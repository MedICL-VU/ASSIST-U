import argparse

'''
Old list
    params = {
        'cameradepth': 14.0,
        'fov': 87 // 2,  # cite boston scientific -- fiberoptic flexs 85
        'localbending': 0,
        'globalbending': 180,
        'wavesize': 10,
        'wavecount': 1,
        'max_depth': 3,
        'renderer': 'unity',
        'numpoints': 1, # how many camera points to sample
        'samplestep': 0.1,
        'visualize': False,
        'savedir': 'output/arpah_unity_smooth',
        'save': True,
        'planmode': 'dfs', # bfs, dfs, unfiltered
        'modelname': 'arpah_decimated',
        'modelpath': 'data/3dmodels/arpah_decimated.stl',
        'models': {'collectingsystem2': (-41.4481495420699, 19.731541937774082, -97.36807944184557),
                   'manualsegmentation1': (-45.75806986508405, 123.49623714056546, 1132.751798267298),
                   'Patient1Right': (23.350248181136337, -137.49344143532295, 803.300038855812),
                   'Patient3Left': (-52.889510869031284, -169.82580547336187, -442.6200299143518),
                   'arpah_decimated':  (-80.8254, -121.328, -1.2019)}
        # arpah (-80.8254, -121.328, -1.2019)
    }
'''

def parse_args():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--cameradepth', default=14.0, type=float)
    parser.add_argument('--fov', default=87 // 2, type=float)
    parser.add_argument('--localbending', default=0, type=int) # 0 will force a single forward point
    parser.add_argument('--globalbending', default=180, type=int) #tree pruning
    parser.add_argument('--wavesize', default=10, type=int)
    parser.add_argument('--wavecount', default= 1, type=int)
    parser.add_argument('--max_depth', default=3, type=int)
    parser.add_argument('--renderer', default='unity', type=str)
    parser.add_argument('--numpoints', default=1, type=int)
    parser.add_argument('--samplestep', default=0.1, type=float)
    parser.add_argument('--visualize', default=True, type=bool)
    parser.add_argument('--savedir', default='output/registered_phantom1', type=str)
    parser.add_argument('--save', default=True, type=bool)
    parser.add_argument('--planmode', default='dfs', type=str) #bfs currently broken
    parser.add_argument('--modelname', default='registered_phantom1', type=str)
    parser.add_argument('--modelpath', default='data/3dmodels/registered_phantom1.stl', type=str)
    parser.add_argument('--contract_mesh', action='store_true')
    parser.add_argument('--smoothing_subdivisions', default=2, type=int)
    parser.add_argument('--smoothing_iter', default=5, type=int)

    parser.add_argument('--recon_mesh', default='data/soft3slowwet/meshed-poisson.ply', type=str)
    parser.add_argument('--recon_images', default='data/soft3slowwet/images.txt', type=str)

    args = parser.parse_args()
    return args