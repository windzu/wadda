from argparse import ArgumentParser


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("function", help="function name want to use")
    parser.add_argument("path", help="file path or dir path", default=None)
    parser.add_argument("--topic", help="ros camera topic")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    if args.function == "pcd" or args.function == "pcd_visualizer":
        from wadda.tools import pcd_visualizer

        pcd_visualizer.main(args)
    elif args.function == "ros" or args.function == "ros_visualizer":
        from wadda.tools import ros_visualizer

        ros_visualizer.main(args)
    elif args.function == "gif" or args.function == "gif_generator":
        from wadda.tools import gif_generator

        gif_generator.main(args)
    else:
        print("function name error")
    return 0
