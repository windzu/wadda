'''
Author: wind windzu1@gmail.com
Date: 2023-08-25 13:46:25
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-27 18:47:25
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
'''
from argparse import ArgumentParser


def parse_args():
    parser = ArgumentParser(
        description=
        "wadda is a collection of toolkits and libraries related to autonomous driving."
    )
    parser.add_argument("function", type=str, help="function name want to use")
    parser.add_argument(
        "path", nargs="?", type=str, help="file path or dir path")
    parser.add_argument(
        "--version", action="version", version="%(prog)s 0.0.1")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    if args.function == "ds" or args.function == "data_slice" or args.function == "data-slice":
        from wadda import data_slice
        data_slice.main(args)

    elif args.function == "gif" or args.function == "gif_generator":
        from wadda import gif_generator

        gif_generator.main(args)
    elif args.function == "v2c" or args.function == "voc2coco":
        from wadda import voc2coco
        voc2coco.main(args)
    else:
        print("function name error")
    return 0
