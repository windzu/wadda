"""
Author: wind windzu1@gmail.com
Date: 2023-08-29 12:00:50
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-29 12:59:07
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
"""
import multiprocessing
import os
import subprocess
import time

import rosbag
import rospy
from pypcd import pypcd
from rich.progress import track

from ..utils.parse_config import parse_compressed_file_list, parse_config
from ..utils.utils import fusion_pcd, ros_to_python_timestamp, save_camera, save_lidar


class Fusion:
    """数据融合
    Args:
        path (str):配置文件路径
    """

    def __init__(self, path="./config.yaml"):
        self.path = path

        self.config = parse_config(path)

        # 使用的进程数
        self.worker_num = self.config["worker_num"]

        # 所有需要解析的文件
        self.files = self.config["files"]

        # 保存的根路径
        self.save_root = self.config["save_root"]

        # 标定信息
        self.cars_calib_info_dict = self.config["cars_calib_info_dict"]

        # 主topic
        self.main_topic = self.config["main_topic"]

        # 所有需要解析的topic
        self.topic_info_list = self.config["topic_info_list"]
        self.topic_list = [info.topic for info in self.topic_info_list]
        self.topic_info_dict = {info.topic: info for info in self.topic_info_list}
        self.lidar_topic_list = [
            info.topic for info in self.topic_info_list if info.sensor_type == "lidar"
        ]

        # topics alias
        self.topics_alias_dict = self.config["topics_alias_dict"]
        self.lidar_frame_id_list = [
            self.topics_alias_dict[topic] for topic in self.lidar_topic_list
        ]

    def fusion(self):
        print("will fusion the pcd files ... ")
        self.fusion_pcd()

    def fusion_pcd(self):
        save_root = self.save_root
        cars_calib_info_dict = self.cars_calib_info_dict

        lidar_frame_id_list = self.lidar_frame_id_list

        # print all car_id
        print("car_id: ")
        for car_id in cars_calib_info_dict:
            print(" ", car_id)

        arguments = []
        for car_id in os.listdir(save_root):
            car_dir = os.path.join(save_root, car_id)
            if os.path.isdir(car_dir) and car_id in cars_calib_info_dict:
                arguments.append(
                    (
                        car_dir,
                        car_id,
                        cars_calib_info_dict,
                        lidar_frame_id_list,
                    )
                )
        with multiprocessing.Pool(processes=self.worker_num) as pool:
            pool.map(self.fusion_pcd_worker, arguments)

    def fusion_pcd_worker(self, args):
        self.merge_lidars_for_car(*args)

    def merge_lidars_for_car(
        self, car_dir, car_id, cars_calib_info_dict, lidar_frame_id_list
    ):
        # get calib dict
        calib_dict = {}
        for frame_id in lidar_frame_id_list:
            calib_info = cars_calib_info_dict[car_id][frame_id]
            calib_dict[frame_id] = calib_info.transform_matrix

        # 循环遍历samples和sweeps目录
        for folder in ["samples", "sweeps"]:
            files_dir = os.path.join(car_dir, folder)
            # judge if the folder exists
            if not os.path.exists(files_dir):
                continue

            lidar_files_dict = {}
            # {
            #     "00001": {
            #         "top_lidar": "path/to/front_center/00001",
            #         "front_lidar": "path/to/front_center/00001",
            #         ...
            #     },
            #     "00002": {
            #         "top_lidar": "path/to/front_center/00002",
            #         "front_lidar": "path/to/front_center/00002",
            #         ...
            # }
            for frame_id in lidar_frame_id_list:
                specific_lidar_dir = os.path.join(files_dir, frame_id)

                # Check if this specific lidar folder exists
                if not os.path.exists(specific_lidar_dir):
                    continue

                for file_name in os.listdir(specific_lidar_dir):
                    file_path = os.path.join(specific_lidar_dir, file_name)

                    if file_name not in lidar_files_dict:
                        lidar_files_dict[file_name] = {}

                    lidar_files_dict[file_name][frame_id] = file_path

        # merge lidar files
        for file_name in track(lidar_files_dict):
            lidar_files = lidar_files_dict[file_name]
            calib_dict = calib_dict

            # get one file path from lidar_files
            file_path = lidar_files[list(lidar_files.keys())[0]]
            parts = file_path.split("/")
            parts[-2] = "fusion_lidar"
            save_path = "/".join(parts)

            # make sure fusion_lidar folder exists
            if not os.path.exists(os.path.dirname(save_path)):
                os.makedirs(os.path.dirname(save_path))

            if len(lidar_files) == len(lidar_frame_id_list):
                fusion_pcd(lidar_files, calib_dict, save_path)

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
