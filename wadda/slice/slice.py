"""
Author: wind windzu1@gmail.com
Date: 2023-08-27 18:34:41
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-29 12:01:45
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""

import multiprocessing
import os
import subprocess

import rosbag
from rich.progress import track

from ..utils.parse_config import parse_compressed_file_list, parse_config
from ..utils.utils import ros_to_python_timestamp, save_camera, save_lidar


class Slice:
    """数据切片
    Args:
        path (str):配置文件路径
    """

    def __init__(self, path="./"):
        self.path = path

        self.worker_num = 4  # default
        self.supported_msg = ["sensor_msgs/CompressedImage", "sensor_msgs/PointCloud2"]

        # 首先解压数据包
        self.decompress(path)

        self.config = parse_config(path)

        # 使用的进程数
        self.worker_num = self.config["worker_num"]

        # 所有需要解析的文件
        self.files = self.config["files"]

        # 数据集根路径
        self.dataset_root = self.config["dataset_root"]

        # 保存的根路径
        self.save_root = self.config["save_root"]

        # 所有需要解析的topic
        # self.topics = self.config["topics"]
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

    def decompress(self, path):
        self.compressed_files = []
        self.compressed_files = parse_compressed_file_list(path)
        # suummary
        print("compressed files: ")
        for file in self.compressed_files:
            print(file)
        print(f"total {len(self.compressed_files)} files")
        print("next decompressing ...")

        # 使用多进程进行解压操作
        with multiprocessing.Pool(processes=self.worker_num) as pool:
            list(
                track(
                    pool.imap_unordered(
                        self.decompress_file_wrapper, enumerate(self.compressed_files)
                    ),
                    total=len(self.compressed_files),
                )
            )

    def decompress_file_wrapper(self, args):
        idx, file = args
        result = self.decompress_file(file)
        return idx, result

    def decompress_file(self, file):
        print(f"Decompressing {file} ...")
        dir_name = os.path.dirname(file)
        base_name = os.path.basename(file).split(".")[0]
        output_dir = os.path.join(dir_name, base_name)

        # 如果输出目录不存在，则创建
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        subprocess.run(["tar", "xzf", file, "-C", output_dir], check=True)

    def slice(self):
        # summary
        print(f"total {len(self.files)} files")

        # 使用多进程进行切片操作
        with multiprocessing.Pool(processes=self.worker_num) as pool:
            list(
                track(
                    pool.imap_unordered(
                        self.process_file_wrapper, enumerate(self.files)
                    ),
                    total=len(self.files),
                )
            )

    def process_file_wrapper(self, args):
        idx, file = args
        result = self.process_file(file)
        return idx, result

    def process_file(self, file):
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

        sweeps_count = 0
        sample_interval = self.sample_interval
        save_sweep_data = self.save_sweep_data

        bag = rosbag.Bag(bag_path)

        data_by_topic = self.preprocess_bag(bag, topic_list + [main_topic])

        # Ensuring that all topics are in data_by_topic dictionary
        for topic in topic_list + [main_topic]:
            if topic not in data_by_topic:
                print(f"Warning: {topic} not found in bag!")
                data_by_topic[topic] = {}

        main_timestamps = list(data_by_topic[main_topic].keys())

        for timestamp in main_timestamps:
            for topic in topic_list:
                closest_time = self.closest_timestamp(
                    timestamp, data_by_topic[topic].keys()
                )
                if abs(closest_time - timestamp) <= time_diff_threshold:
                    msg = data_by_topic[topic][closest_time]

                    topic_info = self.topic_info_dict[topic]
                    topic_str = self.topics_alias_dict[str(topic)]
                    save_sweeps_path = os.path.join(sweeps_path, topic_str)
                    save_samples_path = os.path.join(samples_path, topic_str)
                    filename = str(timestamp)
                    if sweeps_count // sample_interval == 0:
                        self.save_msg_by_topic(
                            msg, topic_info, save_samples_path, filename
                        )
                    if save_sweep_data:
                        self.save_msg_by_topic(
                            msg, topic_info, save_sweeps_path, filename
                        )
            sweeps_count += 1

    def preprocess_bag(self, bag, topics):
        data_by_topic = {}
        for topic, msg, t in bag.read_messages(topics=topics):
            if topic not in data_by_topic:
                data_by_topic[topic] = {}
            python_timestamp = ros_to_python_timestamp(t)
            data_by_topic[topic][python_timestamp] = msg
        return data_by_topic

    def closest_timestamp(self, target_time, timestamps):
        return min(timestamps, key=lambda t: abs(t - target_time))

    def save_msg_by_topic(self, msg, topic_info, path, filename):
        # 根据消息类型选择不同的保存函数
        if topic_info.msg_type == "sensor_msgs/CompressedImage":
            save_camera(msg, topic_info, path, filename)
        elif topic_info.msg_type == "sensor_msgs/PointCloud2":
            # TODO : pypcd parse pointcloud2 is_dense=false data
            save_lidar(msg, topic_info, path, filename)
        else:
            print("msg type not supported")
            print(type(msg))
