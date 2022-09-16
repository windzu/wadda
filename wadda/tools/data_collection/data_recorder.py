from asyncio import constants
import time
import os

# ros
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud2

# local
from .ros_dataset import ROSDataset
from .utils import timestamp_analyze, save_datas, merge_pcd
from .parse_config import *


class DataRecorder:
    """数据记录器
    Args:
        path (str):保存数据的路径 或者 配置文件所在的文件夹路径(此时默认的配置文件名为config.yaml,保存数据的路径也为path)
        pro (bool):是否开启专业模式
    """

    def __init__(self, path="./", pro=False):
        self.path = path
        self.pro = pro

        self.supported_msg = ["sensor_msgs/CompressedImage", "sensor_msgs/PointCloud2"]

        # 1. 初始化topic,确定需要订阅的topic及其对应的消息类型
        self.topic_dict = self.__init_topic(path, pro)

        # 2. 初始化ros_dataset
        self.ros_dataset = ROSDataset(self.topic_dict)

        # 3. 初始化所需常量
        self.total_frames, self.scene_interval = self.__init_constant(path, pro)
        self.scene_interval = rospy.Duration(self.scene_interval)  # float to rospy.Duration()

        # 4. 初始化calib
        self.calib = self.__init_calib(path, pro)

        # 5. 初始化primary_frame_id
        self.primary_frame_id = self.__init_primary_frame_id(path, pro)

    def __init_topic(self, path=None, pro=False):
        """初始化需要订阅的topic

        分为两种模式：
            1. 基础模式:不需要指定topic,根据消息类型检索支持的消息类型
            2. 专业模式:需要指定配置文件和配置所在路径
        Args:
            path (str, optional): 配置文件路径. Defaults to None.
            pro (bool, optional): 是否开启专业模式. Defaults to False.

        Returns:
            dict: 目标的topic和对应的消息类型字典
        """
        rospy.sleep(1)
        all_topics = rospy.get_published_topics()

        topic_dict = {}
        if pro and path:
            # TODO : 根据配置文件解析topic，返回配置文件中指定的topic
            topic_dict = parse_topic(os.path.join(path, "config.yaml"))

        # 如果未开启专业模式或者配置文件中未指定topic,则根据消息类型检索支持的消息类型
        if not topic_dict:
            topic_dict = {}
            for topic in all_topics:
                if topic[1] in self.supported_msg:
                    topic_dict[topic[0]] = topic[1]

        return topic_dict

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

    def __init_primary_frame_id(self, path=None, pro=False):
        """初始化primary_frame_id

        分为两种模式：
            1. 基础模式:使用默认的primary_frame_id
            2. 专业模式:从配置文件中解析获取,需要指定配置文件所在路径
        Args:
            path (str, optional): 配置文件路径. Defaults to None.
            pro (bool, optional): 是否开启专业模式. Defaults to False.
        Returns:
            str: primary_frame_id
        """
        primary_frame_id = None
        if pro and path:
            primary_frame_id = parse_primary_frame_id(os.path.join(path, "config.yaml"))
        return primary_frame_id

    def run(self):
        """开始采集数据"""
        rospy.sleep(1)
        start_time = rospy.Time.now()
        current_time = start_time

        count = 0
        current_scene_name = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime(current_time.secs))

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
                rospy.loginfo("[ DataRecorder ] store the {}th data cost : {}'s ".format(count, end_time - start_time))

                # 更新等待时间并等待
                rospy.sleep(next_time - rospy.Time.now())

                # 保存成功计数加1 并更新时间
                current_time = rospy.Time.now()
                count += 1

            else:
                count = 0
                current_scene_name = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime(current_time.secs))
