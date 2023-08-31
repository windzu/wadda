"""
Author: wind windzu1@gmail.com
Date: 2023-08-27 18:34:41
LastEditors: wind windzu1@gmail.com
LastEditTime: 2023-08-29 11:23:34
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved 
"""
import datetime
import os

import cv2
import numpy as np
import rospy
from pypcd import pypcd

# from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, PointCloud2


def save_camera(msg, topic_info, path, filename):
    file_path = os.path.join(path, filename + ".png")

    if not os.path.exists(os.path.dirname(file_path)):
        os.makedirs(os.path.dirname(file_path))


def save_lidar(msg, topic_info, path, filename):
    file_path = os.path.join(path, filename + ".pcd")

    if not os.path.exists(os.path.dirname(file_path)):
        os.makedirs(os.path.dirname(file_path))

    # save PointCloud2 to bin
    pc = pypcd.PointCloud.from_msg(msg)
    pc.save_pcd(file_path, compression="binary_compressed")
    # pc.save_pcd(file_path, compression="binary")


def save_imu(msg, path, filename):
    pass


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a rotation matrix.
    All angles are in degrees.
    """
    # roll = np.radians(roll)
    # pitch = np.radians(pitch)
    # yaw = np.radians(yaw)

    rotation_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )

    rotation_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    rotation_z = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )

    rotation = np.dot(rotation_z, np.dot(rotation_y, rotation_x))

    return rotation


def timestamp_analyze(datas, main_frame_id):
    """时间戳解析
    Args:
        datas (dict): 一次采样所有的数据, key为frame_id, value为对应的rosmsg
        main_frame_id (str): 用于作为时间参考的frame_id
    """
    main_frame_id_timestamps = datas[main_frame_id].header.stamp
    for key, value in datas.items():
        if key == main_frame_id:
            continue
        else:
            timestamp_diff = main_frame_id_timestamps - value.header.stamp
            # convert to seconds
            timestamp_diff = timestamp_diff.to_sec()
            print(f"{key} timestamp diff: {timestamp_diff}")


def fusion_pcd(datas, calib, save_path):
    """根据calib合并pcd
    Args:
        datas (dict): 一组传感器的数据,其中key为frame_id,value为pcd文件路径
            {
                "front_lidar":"xxx.pcd",
                "left_lidar":"xxx.pcd",
                "right_lidar":"xxx.pcd",
            }
        calib (dict): 传感器之间的标定信息,其中key为frame_id,value为变换矩阵
            {
                "front_lidar":np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),
                "left_lidar":np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),
                "right_lidar":np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),
            }
    Returns:
        dict: datas来自输入datas,但是删除了原始点云,加入了合并后的pcd
    """

    def transform_pcd(pcd, transform):
        """变换pcd

        Args:
            pcd (PointCloud2): pcd
            transform (numpy.ndarray): 变换矩阵

        Returns:
            numpy.ndarray: 变换后的pcd
        """
        pc = pypcd.PointCloud.from_path(pcd)
        x = pc.pc_data["x"].flatten()
        y = pc.pc_data["y"].flatten()
        z = pc.pc_data["z"].flatten()
        intensity = pc.pc_data["intensity"].flatten()
        nan_index = (
            np.isnan(x) | np.isnan(y) | np.isnan(z) | np.isnan(intensity)
        )  # filter nan data
        pc_array_4d = np.zeros((x[~nan_index].shape[0], 4), dtype=np.float32)
        pc_array_4d[:, 0] = x[~nan_index]
        pc_array_4d[:, 1] = y[~nan_index]
        pc_array_4d[:, 2] = z[~nan_index]
        pc_array_4d[:, 3] = 1  # 待会儿要乘以变换矩阵，所以最后一列要是1

        pc_array_4d = np.dot(transform, pc_array_4d.T).T
        pc_array_4d[:, 3] = intensity[~nan_index]  # 变换后的点云的intensity要重新赋值
        return pc_array_4d

    fusion_pcd = None
    for key, value in calib.items():
        # raw_pcd_file = datas[key]
        # read pcd
        # raw_pcd = pypcd.PointCloud.from_path(raw_pcd_file)
        pcd = transform_pcd(datas[key], calib[key])
        if fusion_pcd is None:
            fusion_pcd = pcd
        else:
            fusion_pcd = np.vstack((fusion_pcd, pcd))
    # datas["LIDAR"] = merge_pcd
    structured_pc_array = numpy_array_to_structured_array(fusion_pcd)
    fusion_pc = pypcd.PointCloud.from_array(structured_pc_array)
    fusion_pc.save_pcd(save_path, compression="binary_compressed")


def save_datas(path, scene_name, datas, filename):
    """将数据保存到磁盘
    Args:
        path (str): 保存的根路径
        scene_name (str): 场景名称,用来作为父文件夹名称
        datas (dict): 数据字典,key是子文件夹名称,value是对于的数据
        filename (str): 文件名称,不包括后缀
    TODO :
        - 提高保存数据的效率
    """

    def save_img(path, data, filename):
        buf = np.ndarray(shape=(1, len(data.data)), dtype=np.uint8, buffer=data.data)
        frame = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
        save_path = os.path.join(path, filename + ".jpg")
        ret = cv2.imwrite(save_path, frame)
        if not ret:
            rospy.logwarn("save image failed! path : {}".format(path))
            return False
        return True

    def save_pcd(path, data, filename, suffix=".bin"):
        # TODO :
        # - 增加保存pcd的可行性验证
        x, y, z, intensity = None, None, None, None
        if isinstance(data, PointCloud2):
            pc = pypcd.PointCloud.from_msg(data)
            x = pc.pc_data["x"].flatten()
            y = pc.pc_data["y"].flatten()
            z = pc.pc_data["z"].flatten()
            intensity = pc.pc_data["intensity"].flatten()
        elif isinstance(data, np.ndarray):
            x = data[:, 0]
            y = data[:, 1]
            z = data[:, 2]
            intensity = data[:, 3]
        pc_array_4d = np.zeros((x.shape[0], 4), dtype=np.float32)
        pc_array_4d[:, 0] = x
        pc_array_4d[:, 1] = y
        pc_array_4d[:, 2] = z
        pc_array_4d[:, 3] = intensity

        save_path = os.path.join(path, filename + suffix)
        if suffix == ".pcd":
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(
                np.vstack((pc_array_4d[:, 0], pc_array_4d[:, 1], pc_array_4d[:, 2])).T
            )
            pcd.colors = o3d.utility.Vector3dVector(
                np.vstack((pc_array_4d[:, 3], pc_array_4d[:, 3], pc_array_4d[:, 3])).T
            )
            o3d.io.write_point_cloud(save_path, pcd)
        elif suffix == ".npy":
            path = path.remove(".npy")
            np.save(save_path, pc_array_4d)
        elif suffix == ".bin":
            nan_index = (
                np.isnan(x) | np.isnan(y) | np.isnan(z) | np.isnan(intensity)
            )  # filter nan data
            pc_array_4d = pc_array_4d[~nan_index]
            pc_array_4d.tofile(save_path)
        else:
            raise ValueError("data type is not PointCloud2 or numpy.ndarray")

    def save(path, data, filename):
        # judge data type and choose different save method
        if isinstance(data, CompressedImage):
            save_img(path, data, filename)
        elif isinstance(data, PointCloud2) or isinstance(data, np.ndarray):
            save_pcd(path, data, filename)
        else:
            raise ValueError("data type is not CompressedImage or PointCloud2")

    for frame_id, data in datas.items():
        store_path = os.path.join(path, scene_name, frame_id)
        if not os.path.exists(store_path):
            os.makedirs(store_path)

        save(path=store_path, data=data, filename=filename)


def ros_to_python_timestamp(ros_timestamp):
    dt = datetime.datetime.fromtimestamp(ros_timestamp.secs)
    dt += datetime.timedelta(microseconds=ros_timestamp.nsecs // 1000)
    total_milliseconds = int(dt.timestamp() * 1000)
    return total_milliseconds


def numpy_array_to_structured_array(arr):
    """
    将常规的numpy数组转换为结构化数组

    Args:
        arr (numpy.ndarray): 输入的numpy数组，每一列对应一个字段.

    Returns:
        numpy.ndarray: 结构化数组
    """
    assert arr.shape[1] == 4, "Expected a Nx4 numpy array"
    dtype = [("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4")]
    structured_arr = np.zeros(arr.shape[0], dtype=dtype)

    structured_arr["x"] = arr[:, 0]
    structured_arr["y"] = arr[:, 1]
    structured_arr["z"] = arr[:, 2]
    structured_arr["intensity"] = arr[:, 3]

    return structured_arr
