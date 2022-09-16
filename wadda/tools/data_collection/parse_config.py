import os
import yaml
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R


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
