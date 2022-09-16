from argparse import ArgumentParser

# ros
import rospy

# local
from .data_recorder import DataRecorder


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("--path", type=str, default="./", help="data root path")
    parser.add_argument("--pro", action="store_true", help="whether to enable professional mode")
    return parser.parse_args()


def main(args=None):
    rospy.init_node("data_collection", anonymous=True)
    if args is None:
        args = parse_args()

    # gif generator
    data_recorder = DataRecorder(path=args.path, pro=args.pro)
    data_recorder.run()


if __name__ == "__main__":
    main()
