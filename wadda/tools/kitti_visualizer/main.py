from pathlib import Path
import argparse
import numpy as np
import open3d
import os


class KittiVisualizer:

    def __init__(self, path, width=1024, height=768):
        self.path = path
        self.width = width
        self.height = height

        self.index = 0  # record the index of file_list
        self.file_list = []
        self.axis_pcd = open3d.geometry.TriangleMesh().create_coordinate_frame(
        )

        self.calib = False  # 是否存在calib文件夹
        self.supported_file_type = ["pcd", "bin", "npz"]

        self.__visualizer_init()
        self.__file_init()

    def show(self):
        # 初始化显示pcd第一帧
        if self.file_list:
            self.__update(self.index)

            # 设置键盘响应事件
            self.visualizer.register_key_callback(
                90, lambda temp: self.__last())  # z last
            self.visualizer.register_key_callback(
                ord(" "), lambda temp: self.__next())  # space next
            self.visualizer.register_key_callback(
                ord("q"), lambda temp: self.__close())  # 'q' close
            self.visualizer.run()

    def __file_init(self):
        """遍历文件夹下calib,label_2,velodyne三个文件夹,这些文件夹中的文件名相同,仅仅是文件后缀不同,将文件名保存到self.file_list中
        其中calib文件夹保存的是camera与lidar的相关信息,不是强制的,如果存在calib文件夹则将self.calib置为True
        """
        # 判断path路径下是否存在calib,label_2,velodyne三个文件夹
        if Path(self.path).is_dir():
            self.path = Path(self.path)
            self.calib = Path(self.path, "calib").exists()
            if not Path(self.path, "label_2").exists():
                raise Exception("label_2 not exists")
            if not Path(self.path, "velodyne").exists():
                raise Exception("velodyne not exists")
        else:
            raise Exception("path not exists")

        # 获取velodyne文件夹下所有文件的文件名,并将其排序,保存到self.file_list中
        self.velodyne_path = Path(self.path, "velodyne")
        if Path(self.velodyne_path).is_dir():
            self.file_list = []
            for file in Path(self.velodyne_path).rglob("*"):
                if str(file).split(".")[-1] in self.supported_file_type:
                    self.file_list.append(
                        (str(file).split("/")[-1]).split(".")[0])
            self.file_list = sorted(self.file_list, key=lambda p: Path(p).stem)
        else:
            raise Exception("velodyne not exists")

    def __visualizer_init(self):
        self.visualizer = open3d.visualization.VisualizerWithKeyCallback()
        self.visualizer.create_window(
            window_name="viwer", width=self.width, height=self.height)

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

        # index 是文件名的索引,需要将其转换为文件名，然后拼接成文件路径
        # 需要获取calib,label_2,velodyne文件夹下的文件名
        calib_file = None
        label_file = None
        pcd_file = None
        if self.calib:
            calib_file = str(
                self.path) + "/calib/" + self.file_list[index] + ".txt"
        label_file = str(
            self.path) + "/label_2/" + self.file_list[index] + ".txt"
        pcd_file = str(
            self.path) + "/velodyne/" + self.file_list[index] + ".bin"

        # 读取calib文件,如果存在该文件,则返回从camera到lidar的4x4变换矩阵，否则返回4x4单位矩阵
        # calib = self.__read_calib(calib_file)
        # 读取label文件,如果存在该文件,则返回label信息,否则返回None
        labels = self.__read_label(label_file)
        # 读取pcd文件,如果存在该文件,则返回点云信息,否则返回None
        points = self.__read_pcd(pcd_file)

        if labels is None or points is None:
            print("label or points is None,pass this one")
            return

        self.visualizer.clear_geometries()  # clear first
        o3d_pcd = open3d.geometry.PointCloud(
            open3d.utility.Vector3dVector(points))
        self.axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1, origin=[0, 0, 0])  # draw axis
        self.visualizer.add_geometry(o3d_pcd)
        self.visualizer.add_geometry(self.axis_pcd)

        # debug
        print("labels:", labels)
        # TODO : 结合calib信息进行坐标变换

        for label in labels:
            # set cube
            dimensions = label["dimensions"]  # [l,w,h]
            location = label["location"]  # [x,y,z]
            # 1. dimensions
            cube = open3d.geometry.TriangleMesh.create_box(
                width=dimensions[0], height=dimensions[1], depth=dimensions[2])
            cube.compute_vertex_normals()
            # 2. location
            cube.transform(
                np.array([[1.0, 0, 0.0, float(location[0])],
                          [0.0, 1.0, 0.0, float(location[1])],
                          [0.0, 0.0, 1.0, float(location[2])],
                          [0.0, 0.0, 0.0, 1.0]]))
            # set color to red
            cube.paint_uniform_color([1, 0, 0])

            self.visualizer.add_geometry(cube)

        self.visualizer.poll_events()
        self.visualizer.update_renderer()

    def __read_calib(self, calib_file):
        # 按照kitti的格式读取calib文件
        pass

    def __read_label(self, label_file):
        # 读取label文件,如果存在该文件,则返回label信息,否则返回None
        # label中没一行包含至少15个元素，下面列出在可视化中需要的元素
        # 0:类别，表示该目标的类别，如Car，Pedestrian等
        # 8-10:dimensions, 在标准kitti格式中,表示该目标的高，宽，长
        # 11-13:location, 在标准kitti格式中,表示该目标的bbox底面的中心点在所处的坐标系中的x,y,z的坐标
        # # (有calib时为camera坐标系，无calib时为lidar坐标系)
        # 14:rotation_y, 在标准kitti格式中,表示该目标的bbox底面的中心点在所处的坐标系中相较于x轴的旋转角度
        if os.path.exists(label_file):
            labels = []
            with open(label_file, "r") as f:
                for line in f.readlines():
                    line = line.strip()
                    if len(line) >= 15:
                        class_name = line.split(" ")[0]
                        dimensions = [
                            float(line.split(" ")[8]),
                            float(line.split(" ")[9]),
                            float(line.split(" ")[10])
                        ]
                        location = [
                            float(line.split(" ")[11]),
                            float(line.split(" ")[12]),
                            float(line.split(" ")[13])
                        ]
                        rotation_y = float(line.split(" ")[14])
                        label = dict()
                        label["class_name"] = class_name
                        # filter out the class_name is DontCare
                        if label["class_name"] == "DontCare":
                            continue
                        label["dimensions"] = dimensions
                        label["location"] = location
                        label["rotation_y"] = rotation_y
                        labels.append(label)
            return labels
        else:
            return None

    def __read_pcd(self, pcd_file):
        # 读取pcd文件,如果存在该文件,则返回点云信息,否则返回None
        # TODO : 解决维度不确定的问题
        bin_pcd = np.fromfile(pcd_file, dtype=np.float32)

        # can not sure the shape of bin_pcd is (n, 4) or (n, 5)
        # first think it is (n, 5)

        # if bin_pcd.shape[0] % 5 == 0:
        #     points = bin_pcd.reshape((-1, 5))[:, 0:3]
        # elif bin_pcd.shape[0] % 4 == 0:
        #     points = bin_pcd.reshape((-1, 4))[:, 0:3]
        # else:
        #     print("bin_pcd shape error")
        #     return None

        # debug
        points = bin_pcd.reshape((-1, 4))[:, 0:3]
        return points


def parse_args():
    parser = argparse.ArgumentParser(description="view point cloud")
    parser.add_argument("--path", help="file path or dir path")
    parser.add_argument("--width", help="window width", default=1024)
    parser.add_argument("--height", help="window height", default=768)
    return parser.parse_args()


def main(args=None):
    if args is None:
        args = parse_args()
    viewer = KittiVisualizer(path=args.path)
    viewer.show()


if __name__ == "__main__":
    main()
