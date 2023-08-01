import os
import glob
import cv2
import numpy as np


def genMask():
    mask = np.ones((512,512,3))
    TRIANGLE_LENGTH = 85
    trimaskr = np.array(np.triu(np.ones((512,512)), 512 - TRIANGLE_LENGTH -1), dtype=bool)
    trimaskl = np.fliplr(trimaskr)
    mask[trimaskr] = [0,0,0]
    mask[trimaskr.T] = [0,0,0]
    mask[trimaskl] = [0,0,0]
    mask[np.fliplr(trimaskr.T)] = [0,0,0]
    return mask

def cornerBlock(fpath):
    for fname in glob.glob(fpath + '/*'):
        root, ftype = os.path.splitext(fname)
        path = os.path.normpath(root)
        img =cv2.imread(fname)
        resized = cv2.resize(img, (512, 512))
        mask = genMask()
        new_img = resized * mask
        # new_img = cv2.resize(new_img, (256, 256))
        cv2.imwrite(fname, new_img)