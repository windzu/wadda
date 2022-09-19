from argparse import ArgumentParser


def parse_args():
    parser = ArgumentParser(
        description="wadda is a collection of toolkits and libraries related to autonomous driving."
    )
    parser.add_argument("function", type=str, help="function name want to use")
    parser.add_argument("path", nargs="?", type=str, help="file path or dir path")
    parser.add_argument("--pro", action="store_true", help="whether to enable professional mode")
    parser.add_argument("--version", action="version", version="%(prog)s 0.0.1")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    if args.function == "dc" or args.function == "data_collection":
        from wadda.tools import data_collection

        data_collection.main(args)

    elif args.function == "pcd" or args.function == "pcd_visualizer":
        from wadda.tools import pcd_visualizer

        pcd_visualizer.main(args)
    elif args.function == "gif" or args.function == "gif_generator":
        from wadda.tools import gif_generator

        gif_generator.main(args)
    elif args.function == "ros" or args.function == "ros_visualizer":
        from wadda.tools import ros_visualizer

        ros_visualizer.main(args)
    elif args.function == "v2c" or args.function == "voc2coco":
        from wadda.tools import voc2coco

        voc2coco.main(args)

    else:
        print("function name error")
    return 0
