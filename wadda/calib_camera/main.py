"""
Author: wind windzu1@gmail.com
Date: 2023-08-31 16:05:20
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-31 16:08:35
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""
"""
Author: wind windzu1@gmail.com
Date: 2023-08-27 18:34:41
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-29 12:06:39
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""
"""
Author: wind windzu1@gmail.com
Date: 2023-08-27 18:34:41
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-28 19:14:33
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""
import os
from argparse import ArgumentParser

import cv2

from .calibrator import CAMERA_MODEL, Calibrator, ChessboardInfo, MonoCalibrator


def parse_args(argv):
    parser = ArgumentParser()
    parser.add_argument(
        "--camera_model",
        type=str,
        default="pinhole",
        help="pinhole or fisheye [default: pinhole]",
    )
    parser.add_argument(
        "--size",
        type=str,
        default="8x6",
        help="specify chessboard size as NxM [default: 8x6]",
    )
    parser.add_argument(
        "--square",
        type=str,
        default="0.108",
        help="specify chessboard square size in meters [default: 0.108]",
    )
    parser.add_argument(
        "--image_dir",
        type=str,
        default="./images",
        help="specify path to images [default: ./images]",
    )
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)

    # init camera model
    if args.camera_model == "pinhole":
        camera_model = CAMERA_MODEL.PINHOLE
    elif args.camera_model == "fisheye":
        camera_model = CAMERA_MODEL.FISHEYE

    # init chessboard
    boards = []
    info = ChessboardInfo()
    size = tuple([int(c) for c in args.size.split("x")])
    info.dim = float(args.square)
    info.n_cols = size[0]
    info.n_rows = size[1]
    boards.append(info)

    # init calib_flags
    calib_flags = 0
    calib_flags |= cv2.CALIB_FIX_K3

    # get all images path
    image_path_list = []
    for root, dirs, files in os.walk(args.image_dir):
        for file in files:
            if file.endswith(".jpg") or file.endswith(".png"):
                image_path_list.append(os.path.join(root, file))

    calibrator = MonoCalibrator(camera_model, boards, calib_flags)
    calibrator.do_file_calibration(image_path_list)
    result = calibrator.yaml()

    # save result
    with open("result.yaml", "w") as f:
        f.write(result)
