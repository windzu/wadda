"""
Author: wind windzu1@gmail.com
Date: 2023-08-31 16:03:25
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-31 17:29:49
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""
import sys
from argparse import ArgumentParser


def parse_args():
    parser = ArgumentParser(
        description="wadda is a collection of toolkits and libraries related to autonomous driving."
    )
    parser.add_argument("function", type=str, help="function name want to use")
    parser.add_argument("--version", action="version", version="%(prog)s 0.0.1")
    args, _ = parser.parse_known_args()
    # parser.add_argument("path", nargs="?", type=str, help="file path or dir path")
    # parser.add_argument(
    #     "--camera_model",
    #     type=str,
    #     default="pinhole",
    #     help="pinhole or fisheye [default: pinhole]",
    # )
    # parser.add_argument(
    #     "--size",
    #     type=str,
    #     default="8x6",
    #     help="specify chessboard size as NxM [default: 8x6]",
    # )
    # parser.add_argument(
    #     "--square",
    #     type=str,
    #     default="0.108",
    #     help="specify chessboard square size in meters [default: 0.108]",
    # )
    # parser.add_argument(
    #     "--image_dir",
    #     type=str,
    #     default="./images",
    #     help="specify path to images [default: ./images]",
    # )
    # parser.add_argument("--version", action="version", version="%(prog)s 0.0.1")
    # args = parser.parse_args()
    return args


def main():
    args = parse_args()
    if (
        args.function == "s"
        or args.function == "slice"
        or args.function == "data_slice"
        or args.function == "data-slice"
    ):
        from wadda import slice

        slice.main(args)
    elif (
        args.function == "cc"
        or args.function == "calib-camera"
        or args.function == "calib_camera"
    ):
        from wadda import calib_camera

        calib_camera.main(sys.argv[2:])
    elif args.function == "fusion" or args.function == "gif_generator":
        from wadda import fusion

        fusion.main(sys.argv[2:])
    elif args.function == "gif" or args.function == "gif_generator":
        from wadda import gif_generator

        gif_generator.main(sys.argv[2:])
    elif args.function == "v2c" or args.function == "voc2coco":
        from wadda import voc2coco

        voc2coco.main(sys.argv[2:])
    else:
        print("function name error")
    return 0
