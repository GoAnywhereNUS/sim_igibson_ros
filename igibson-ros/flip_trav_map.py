import argparse
import os
import cv2
import numpy as np

def flip(img_path):
    dirname = os.path.dirname(img_path)
    basename = os.path.basename(img_path)
    filename, ext = os.path.splitext(basename)
    modified_basename = filename + '_flipped' + ext
    im = cv2.imread(img_path)
    cv2.imwrite(os.path.join(dirname, modified_basename), im[::-1, :, :])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Flip Traversability Map")
    parser.add_argument("--img_path", type=str, help="Path to load map from", default="")
    args = parser.parse_args()
    flip(args.img_path)