"""
Author: wind windzu1@gmail.com
Date: 2023-08-27 18:34:41
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-28 00:05:50
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""

import os
import subprocess
import time
from asyncio import constants

import rosbag
import rospy
from rich.progress import track
from sensor_msgs.msg import CompressedImage, PointCloud2

from .parse_config import parse_compressed_file_list, parse_config
from .ros_dataset import ROSDataset
from .utils import *


class DataSlice:
    """数据切片
    Args:
        path (str):配置文件路径
    """

    def __init__(self, path="./"):
        self.path = path

        self.supported_msg = ["sensor_msgs/CompressedImage", "sensor_msgs/PointCloud2"]

        # 首先解压数据包
        self.decompress(path)

        self.config = parse_config(path)

        # 所有需要解析的文件
        self.files = self.config["files"]

        # 数据集根路径
        self.dataset_root = self.config["dataset_root"]

        # 保存的根路径
        self.save_root = self.config["save_root"]

        # 所有需要解析的topic
        self.topics = self.config["topics"]
        self.topic_info_list = self.config["topic_info_list"]
        self.topic_list = [info.topic for info in self.topic_info_list]
        self.topic_info_dict = {info.topic: info for info in self.topic_info_list}

        # topics alias
        self.topics_alias_dict = self.config["topics_alias_dict"]

        # 主topic
        self.main_topic = self.config["main_topic"]

        # 时间戳阈值
        self.time_diff_threshold = self.config["time_diff_threshold"]

        # 采样间隔
        self.sample_interval = self.config["sample_interval"]

        # 是否保存sweep数据
        self.save_sweep_data = self.config["save_sweep_data"]

        # 2. 初始化ros_dataset
        # self.ros_dataset = ROSDataset(self.topic_dict)

        # 3. 初始化所需常量
        # self.total_frames, self.scene_interval = self.__init_constant(path, pro)
        # self.scene_interval = rospy.Duration(self.scene_interval)  # float to rospy.Duration()

        # 4. 初始化calib
        # self.calib = self.__init_calib(path, pro)

    def __init_constant(self, path=None, pro=False):
        """初始化必要的常量

        分为两种模式：
            1. 基础模式:全部使用默认值
            2. 专业模式:从配置文件中解析获取,需要指定配置文件所在路径
        Args:
            path (str, optional): 配置文件路径. Defaults to None.
            pro (bool, optional): 是否开启专业模式. Defaults to False.
        Returns:
            tuple: 常量元组,其内容下:
                1. total_frames(int):每一个scene需要采样的总帧数
                2. scene_interval(float):每一个frame之间的时间间隔
        """
        constants = None
        if pro and path:
            constants = parse_constants(os.path.join(path, "config.yaml"))

        # 如果未开启专业模式或者配置文件中未指定topic,则使用默认constants
        if not constants:
            constants = (40, 0.5)  # 默认一个scene采样40帧,每一帧之间的时间间隔为0.5s
        return constants

    def __init_calib(self, path=None, pro=False):
        """初始化calib

        分为两种模式：
            1. 基础模式:使用默认的calib
            2. 专业模式:从配置文件中解析获取,需要指定配置文件所在路径
        Args:
            path (str, optional): 配置文件路径. Defaults to None.
            pro (bool, optional): 是否开启专业模式. Defaults to False.
        Returns:
            dict: calib字典
        """
        calib = None
        if pro and path:
            calib = parse_calib(os.path.join(path, "config.yaml"))
        return calib

    def decompress(self, path):
        compressed_files = []
        compressed_files = parse_compressed_file_list(path)
        # suummary
        print("compressed files: ")
        for file in compressed_files:
            print(file)
        print(f"total {len(compressed_files)} files")
        print("next decompressing ...")
        for file in track(compressed_files):
            print(f"Decompressing {file} ...")
            # 解压至同一目录下
            dir_name = os.path.dirname(file)
            base_name = os.path.basename(file).split(".")[0]
            output_dir = os.path.join(dir_name, base_name)

            # 如果输出目录不存在，则创建
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            subprocess.run(["tar", "xzf", file, "-C", output_dir], check=True)

    def slice(self):
        # summary
        print("files: ")
        for file in self.files:
            print(file)
        print(f"total {len(self.files)} files")
        print("next slicing ...")
        for file in track(self.files):
            print(f"Slicing {file} ...")
            self.extract_data_from_bag(
                file,
                self.dataset_root,
                self.save_root,
                self.topic_list,
                self.main_topic,
                self.time_diff_threshold,
            )

    def extract_data_from_bag(
        self,
        bag_path,
        dataset_root,
        save_root,
        topic_list,
        main_topic,
        time_diff_threshold,
    ):
        car_name = os.path.basename(bag_path).split("_")[0]
        save_path = os.path.join(save_root, car_name)

        samples_path = os.path.join(save_path, "samples")
        sweeps_path = os.path.join(save_path, "sweeps")

        bag = rosbag.Bag(bag_path)

        sweeps_count = 0
        sample_interval = self.sample_interval
        save_sweep_data = self.save_sweep_data
        main_topic_timestamps = []
        for _, _, t in bag.read_messages(topics=[main_topic]):
            main_topic_timestamps.append(t)

        for timestamp in main_topic_timestamps:
            for topic, msg, t in bag.read_messages(topics=topic_list):
                if abs((t - timestamp).to_sec()) <= time_diff_threshold:
                    topic_info = self.topic_info_dict[topic]
                    # sensor_type_str=str(topic_info.sensor_type)
                    topic_str = self.topics_alias_dict[str(topic)]
                    sweeps_path = os.path.join(sweeps_path, topic_str)
                    samples_path = os.path.join(samples_path, topic_str)
                    filename = str(timestamp.to_nsec())
                    if sweeps_count // sample_interval == 0:
                        self.save_msg_by_topic(msg, topic_info, samples_path, filename)
                    if save_sweep_data:
                        self.save_msg_by_topic(msg, topic_info, sweeps_path, filename)

                    sweeps_count += 1

    def save_msg_by_topic(self, msg, topic_info, path, filename):
        # 根据消息类型选择不同的保存函数
        if topic_info.msg_type == "sensor_msgs/CompressedImage":
            save_camera(msg, topic_info, path, filename)
        elif topic_info.msg_type == "sensor_msgs/PointCloud2":
            save_lidar(msg, topic_info, path, filename)
        else:
            print("msg type not supported")
            print(type(msg))

    def run(self):
        """开始采集数据"""
        rospy.sleep(1)
        start_time = rospy.Time.now()
        current_time = start_time

        count = 0
        current_scene_name = time.strftime(
            "%Y_%m_%d_%H_%M_%S", time.localtime(current_time.secs)
        )

        while True:
            # TODO :
            # - 增加使用独立线程将数据写入磁盘，以免影响数据采集的进程
            if count < self.total_frames:
                datas = self.ros_dataset.get_data(current_time)
                if datas is None:
                    # 获取的数据无效
                    rospy.sleep(self.scene_interval)
                    current_time = rospy.Time.now()
                    continue

                # 如果设置了 primary_frame_id ，则以 primary_frame_id 数据的时间戳作为触发采样时间
                if self.primary_frame_id:
                    current_time = datas[self.primary_frame_id].header.stamp
                next_time = current_time + self.scene_interval  # 计算下一个需要采样的时间点

                # 如果设置了标定参数，则对点云进行拼接
                if self.calib:
                    datas = merge_pcd(datas, self.calib)

                # 存储数据,并打印存储消耗的时间
                start_time = time.time()
                save_datas(
                    path=self.path,
                    scene_name=current_scene_name,
                    datas=datas,
                    filename=str(current_time.to_nsec()),
                )
                end_time = time.time()
                rospy.loginfo(
                    "[ DataRecorder ] store the {}th data cost : {}'s ".format(
                        count, end_time - start_time
                    )
                )

                # 更新等待时间并等待
                rospy.sleep(next_time - rospy.Time.now())

                # 保存成功计数加1 并更新时间
                current_time = rospy.Time.now()
                count += 1

            else:
                count = 0
                current_scene_name = time.strftime(
                    "%Y_%m_%d_%H_%M_%S", time.localtime(current_time.secs)
                )
