from pathlib import Path
import argparse
import numpy as np
import open3d


class PCDVisualizer:
    def __init__(self, file_path, width=1024, height=768):
        self.file_path = file_path
        self.width = width
        self.height = height

        self.index = 0  # record the index of file_list
        self.file_list = []
        self.axis_pcd = open3d.geometry.TriangleMesh().create_coordinate_frame()

        self.supported_file_type = ["pcd", "bin"]

        self.__visualizer_init()
        self.__file_init()

    def show(self):
        # 初始化显示pcd第一帧
        if self.file_list:
            self.__update(self.index)

            # 设置键盘响应事件
            self.visualizer.register_key_callback(90, lambda temp: self.__last())  # z last
            self.visualizer.register_key_callback(ord(" "), lambda temp: self.__next())  # space next
            self.visualizer.register_key_callback(ord("q"), lambda temp: self.__close())  # 'q' close
            self.visualizer.run()

    def __file_init(self):
        if Path(self.file_path).is_dir():
            self.file_list = []
            for file in Path(self.file_path).rglob("*"):
                if str(file).split(".")[-1] in self.supported_file_type:
                    self.file_list.append(str(file))
            self.file_list = sorted(self.file_list, key=lambda p: Path(p).stem)
        elif Path(self.file_path).is_file() and self.file_path.split(".")[-1] in self.supported_file_type:
            self.file_list = [self.file_path]

    def __visualizer_init(self):
        self.visualizer = open3d.visualization.VisualizerWithKeyCallback()
        self.visualizer.create_window(window_name="viwer", width=self.width, height=self.height)

        # setting
        visualizer_option = self.visualizer.get_render_option()
        # 设置背景颜色和点大小
        visualizer_option.background_color = np.asarray([0, 0, 0])
        visualizer_option.point_size = 2

    def __next(self):
        if 0 <= self.index < len(self.file_list) - 1:
            self.index += 1
            self.__update(self.index)
        else:
            print("the last one")

    def __last(self):
        # debug
        print("enter last")

        if 0 < self.index <= len(self.file_list) - 1:
            self.index -= 1
            self.__update(self.index)
        else:
            print("the first one")

    def __close(self):
        self.visualizer.destroy_window()

    def __update(self, index):
        new_pcd_file = self.file_list[index]
        self.visualizer.clear_geometries()  # clear first
        if new_pcd_file.endswith(".pcd"):
            o3d_pcd = open3d.io.read_point_cloud(new_pcd_file)
        elif new_pcd_file.endswith(".bin"):
            bin_pcd = np.fromfile(new_pcd_file, dtype=np.float32)
            points = bin_pcd.reshape((-1, 4))[:, 0:3]
            o3d_pcd = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(points))
        else:
            print("suffix error")
            return
        self.visualizer.add_geometry(o3d_pcd)
        self.visualizer.add_geometry(self.axis_pcd)
        self.visualizer.poll_events()
        self.visualizer.update_renderer()


def parse_args():
    parser = argparse.ArgumentParser(description="view point cloud")
    parser.add_argument("-path", help="file path or dir path")
    parser.add_argument("--width", help="window width", default=1024)
    parser.add_argument("--height", help="window height", default=768)
    return parser.parse_args()


def main(args=None):
    if args is None:
        args = parse_args()
    viewer = PCDVisualizer(file_path=args.path)
    viewer.show()


if __name__ == "__main__":
    main()
