import os
import yaml
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# ros
import rospy
from pypcd import pypcd
from sensor_msgs.msg import CompressedImage, PointCloud2


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


def merge_pcd(datas, calib):
    """根据calib合并pcd
    Args:
        datas (dict): 一组传感器的数据,其中key为frame_id,value为具体数据
        calib (dict): 传感器之间的标定信息,其中key为frame_id,value为变换矩阵
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
        pc = pypcd.PointCloud.from_msg(pcd)
        x = pc.pc_data["x"].flatten()
        y = pc.pc_data["y"].flatten()
        z = pc.pc_data["z"].flatten()
        intensity = pc.pc_data["intensity"].flatten()
        nan_index = np.isnan(x) | np.isnan(y) | np.isnan(z) | np.isnan(intensity)  # filter nan data
        pc_array_4d = np.zeros((x[~nan_index].shape[0], 4), dtype=np.float32)
        pc_array_4d[:, 0] = x[~nan_index]
        pc_array_4d[:, 1] = y[~nan_index]
        pc_array_4d[:, 2] = z[~nan_index]
        pc_array_4d[:, 3] = 1  # 待会儿要乘以变换矩阵，所以最后一列要是1

        pc_array_4d = np.dot(transform, pc_array_4d.T).T
        pc_array_4d[:, 3] = intensity[~nan_index]  # 变换后的点云的intensity要重新赋值
        return pc_array_4d

    merge_pcd = None
    for key, value in calib.items():
        assert key in datas, f"calib key {key} not in datas"
        assert isinstance(datas[key], PointCloud2), f"calib key {key} not PointCloud2"
        pcd = transform_pcd(datas[key], calib[key])
        del datas[key]
        if merge_pcd is None:
            merge_pcd = pcd
        else:
            merge_pcd = np.vstack((merge_pcd, pcd))
    datas["LIDAR"] = merge_pcd
    return datas


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
            nan_index = np.isnan(x) | np.isnan(y) | np.isnan(z) | np.isnan(intensity)  # filter nan data
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
