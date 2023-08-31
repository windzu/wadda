"""
Author: wind windzu1@gmail.com
Date: 2023-08-29 12:00:50
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-29 12:01:05
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

from argparse import ArgumentParser

from .fusion import Fusion


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("--path", type=str, default="./config.yaml", help="config path")
    return parser.parse_args()


def main(args=None):
    if args is None:
        args = parse_args()

    # gif generator
    fusion = Fusion(path=args.path)
    fusion.fusion()


if __name__ == "__main__":
    main()
