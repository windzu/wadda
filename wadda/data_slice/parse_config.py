'''
Author: wind windzu1@gmail.com
Date: 2023-08-25 13:59:36
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-25 19:42:24
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
'''
import os

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


# from scipy.spatial.transform import Rotation as R
class TopicInfo:
    def __init__(self, topic, sensor_type,msg_type):
        self.topic = topic
        self.sensor_type = sensor_type
        self.msg_type = msg_type
    def __repr__(self):
        return f"TopicInfo(topic={self.topic}, sensor_type={self.sensor_type}, msg_type={self.msg_type})"

class CalibInfo:
    def __init__(self, frame_id, tf_config):
        self.frame_id = frame_id
        self.transform_matrix = self.get_transform_matrix(tf_config)

    def get_transform_matrix(self,tf_config):
        """获取transform matrix
        Returns:
            np.ndarray: 4x4的transform matrix
        """
        transform = np.eye(4)
        # rotation
        tf_x=tf_config["tf_x"]
        tf_y=tf_config["tf_y"]
        tf_z=tf_config["tf_z"]
        tf_roll=tf_config["tf_roll"]
        tf_pitch=tf_config["tf_pitch"]
        tf_yaw=tf_config["tf_yaw"]
        r = R.from_euler('xyz', [tf_roll, tf_pitch, tf_yaw], degrees=True)
        rotation = r.as_matrix()
        # translation
        translation = np.array([tf_x,tf_y,tf_z]).reshape(3, 1)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation.flatten()

        return transform
    def __repr__(self):
        return f"CalibInfo(frame_id={self.frame_id}, transform_matrix={self.transform_matrix})"

# 读取配置文件
def read_config(path="./config.yaml"):
    with open(path, 'r') as file:
        config = yaml.safe_load(file)
    return config

# 读取文件列表
def parse_file_list(path="./config.yaml"):
    config = read_config(path)

    # 获取配置值
    dataset_root = config['dataset_root']
    exclude_path = config['exclude_path']
    suffix = config['suffix']


    def get_files_from_directory(directory, suffix, excluded_paths):
        file_list = []
        
        # 遍历指定目录
        for dirpath, dirnames, filenames in os.walk(directory):
            # 检查当前目录是否在排除列表中
            if dirpath not in excluded_paths:
                for filename in filenames:
                    if filename.endswith(suffix):
                        file_list.append(os.path.join(dirpath, filename))
        return file_list
    
    files = get_files_from_directory(dataset_root, suffix, exclude_path)
    return files

# 读取数据根路径
def parse_dataset_root(path="./config.yaml"):
    config = read_config(path)

    # 获取配置中的topics
    dataset_root = config['dataset_root']
    return dataset_root

# 读取保存根路径
def parse_save_root(path="./config.yaml"):
    config = read_config(path)

    # 获取配置中的topics
    save_root = config['save_root']
    return save_root

# 读取topic列表
def parse_topic_list(path="./config.yaml"):
    config = read_config(path)

    # 获取配置中的topics
    topic_data = config['topics']


    def extract_topics(data):
        topics = []
        if isinstance(data, dict):
            for key, value in data.items():
                if value:
                    topics.extend(extract_topics(value))
                else:
                    topics.append(key)
        elif isinstance(data, list):
            for item in data:
                topics.extend(extract_topics(item))
        else:
            topics.append(data)
        return topics
    topics = extract_topics(topic_data)
    return topics

def parse_topic_infos(path="./config.yaml"):
    config = read_config(path)

    topic_data = config['topics']
    msg_type_data = config['msg_type']

    topic_infos = []

    def extract_topics(data, sensor_type):
        topics = []
        if isinstance(data, dict):
            for key, value in data.items():
                if value:
                    # 如果值不为空（例如，lidar），则递归提取
                    topics.extend(extract_topics(value, key))
                else:
                    # 否则，这是一个主题名，创建TopicInfo对象
                    topics.append(TopicInfo(topic=key, sensor_type=sensor_type, msg_type=msg_type_data[key]))
        elif isinstance(data, list):
            for item in data:
                # 如果值是列表，为列表中的每个主题创建TopicInfo对象
                topics.append(TopicInfo(topic=item, sensor_type=sensor_type, msg_type=msg_type_data[sensor_type]))
        else:
            # 否则，这是一个主题名，创建TopicInfo对象
            topics.append(TopicInfo(topic=data, sensor_type=sensor_type, msg_type=msg_type_data[sensor_type]))
        return topics

    topic_infos = extract_topics(topic_data, None)
    return topic_infos

# 读取 topic 的别名
def parse_topics_alias(path="./config.yaml"):
    config = read_config(path)

    # 获取配置中的topics
    topics_alias_dict = config['topics_alias']
    return topics_alias_dict

# 读取主topic
def parse_main_topic(path):
    config = read_config(path)

    # 获取配置中的topics
    main_topic = config['main_topic']
    return main_topic

# 读取标定信息
def parse_calib(path="./config.yaml"):
    config = read_config(path)
    calibs={}

    # 获取配置中的topics
    calib = config['calib']
    load_way=calib['load_way']
    if(load_way=='offline'):
        print("will load calib from offline file")
        calib_path=calib['calib_path']
        # 便利该文件夹下所有yaml文件
        calib_files = []
        for dirpath, dirnames, filenames in os.walk(calib_path):
            for filename in filenames:
                if filename.endswith('.yaml'):
                    calib_files.append(os.path.join(dirpath, filename))
        # 读取yaml文件
        
        for calib_file in calib_files:
            car_calib_info={}
            car_name=os.path.basename(calib_file).split('.')[0]
            with open(calib_file, 'r') as file:
                calib_data = yaml.safe_load(file)
                # calib[car_name]=CalibInfo(frame_id=calib_data['frame_id'],tf_config=calib_data['tf_config'])
    else:
        print("will load calib from config")
        calib_path=None
        calib=None
    return calib


# 读取时间戳阈值
def parse_time_diff_threshold(path):
    config = read_config(path)

    # 获取配置中的topics
    time_diff_threshold = config['time_diff_threshold']
    return time_diff_threshold

def parse_topic(path="./config.yaml"):
    """从config文件中解析rostopic
    Args:
        path (str, optional): 配置文件路径. Defaults to "./config.yaml".
    Returns:
        dict: 目标的topic和对应的消息类型字典
    """
    # judge file exist
    if not os.path.exists(path):
        return None

    with open(path, "r") as f:
        config = yaml.safe_load(f)
    if "topic" not in config:
        print("topic not in config")
        return None

    topic_dict = config["topic"]
    return topic_dict


def parse_constants(path="./config.yaml"):
    """从config文件中解析常量
    Args:
        path (str, optional): 配置文件路径. Defaults to "./config.yaml".
    Returns:
        tuple: 常量元组,其内容下:
            1. total_frames(int):每一个scene需要采样的总帧数
            2. scene_interval(float):每一个frame之间的时间间隔
    """
    # judge file exist
    if not os.path.exists(path):
        return None

    with open(path, "r") as f:
        config = yaml.safe_load(f)
    if "constants" not in config:
        print("constants not in config")
        return None

    constants_dict = config["constants"]
    total_frames = int(constants_dict["total_frames"])
    scene_interval = float(constants_dict["scene_interval"])
    return total_frames, scene_interval


def parse_calib(path="./config.yaml"):
    """从config文件中解析外參参数
    Args:
        path (str, optional): 配置文件路径. Defaults to "./config.yaml".
    Returns:
        dict: 每个传感器的外参字典,key为传感器的frame_id,value为对应的外参矩阵
    """
    # judge file exist
    if not os.path.exists(path):
        return None

    with open(path, "r") as f:
        config = yaml.safe_load(f)
    if "calib" not in config:
        print("calib not in config")
        return None

    calib_dict = config["calib"]

    calib = {}
    for frame_id, transform_dict in calib_dict.items():
        current_transform = np.eye(4)

        # rotation
        r = R.from_quat(transform_dict["rotation"])
        rotation = r.as_matrix()
        current_transform[:3, :3] = rotation

        # translation
        translation = np.array(transform_dict["translation"]).reshape(3, 1)
        current_transform[:3, 3] = translation.flatten()

        calib[frame_id] = current_transform

    return calib


def parse_primary_frame_id(path="./config.yaml"):
    """从config文件中解析主frame_id
    Args:
        path (str, optional): 配置文件路径. Defaults to "./config.yaml".
    Returns:
        str: 主frame_id
    """
    # judge file exist
    if not os.path.exists(path):
        return None

    with open(path, "r") as f:
        config = yaml.safe_load(f)
    if "primary_frame_id" not in config:
        print("primary_frame_id not in config")
        return None

    primary_frame_id = config["primary_frame_id"]
    return primary_frame_id
