"""
Author: wind windzu1@gmail.com
Date: 2023-08-27 18:34:41
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-28 00:18:00
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""
import sys
from argparse import ArgumentParser

import rospy

# local
from .data_slice import DataSlice


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("--path", type=str, default="./config.yaml", help="config path")
    return parser.parse_args()


def main(args=None):
    if args is None:
        args = parse_args()

    # gif generator
    data_slice = DataSlice(path=args.path)
    data_slice.slice()


if __name__ == "__main__":
    main()