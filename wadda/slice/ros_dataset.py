import threading
from queue import Queue

# ros
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud2


class ROSDataset:
    """启用独立线程订阅topic list,维护一个固定长度的存储队列
    Args:
        topic_dict(dict): topic字典,key为topic名称,value为msg数据类型
        queue_size(int): 存储队列的长度
        offline(bool): 是否离线模式,如果是离线模式,则以系统时间作为时间戳
    """

    def __init__(self, topic_dict, queue_size=10, offline=False):
        self.topic_dict = topic_dict
        self.queue_size = queue_size
        self.offline = offline

        self.frame_id_list = []  # 存储所有topic的frame_id，frame_id是唯一的
        self.subscriber_list = []  # 存储所有的subscriber
        self.data_queue_dict = {}  # 存储所有topic的一段时间内数据

        # start collect data thread
        self.t = threading.Thread(target=self.run)
        self.t.start()

    def get_data(self, current_time=None):
        """获取与current_time最近的所有数据
        Args:
            current_time(rospy.Time): 给定的时间,如果为None,则使用系统时间
        Returns:
            data(dict): key为frame_id名称,value为msg数据
        """
        if current_time is None:
            current_time = rospy.Time.now()
        data = {}
        for frame_id in self.frame_id_list:
            current_frame = self.__get_the_closest_frame(current_time, frame_id)
            if current_frame is not None:
                data[frame_id] = current_frame
            else:
                return None
        return data

    def __get_the_closest_frame(self, current_time, frame_id):
        """获取队列中时间最接近的数据"""
        if not self.data_queue_dict[frame_id].empty():
            # 遍历队列,找到时间最接近的数据
            # 因为队列中的数据是按时间顺序排列的,所以只需要遍历一遍即可
            # 最近的数据分几种存在位置
            # - 如果队列长度为1,则这个数据就是最近的数据
            # - 如果队列长度大于1,则需要遍历队列,找到时间最接近的数据
            # - - 如果出现在队列中间位置，需要比较前后两个数据的时间差，找到最接近的数据
            # - - 如果出现在队列开头位置，则返回队列第一个数据
            # - - 如果出现在队列结尾位置，则返回队列最后一个数据
            # - 如果队列长度为0,则返回None
            # TODO ： 目前的检查不够完备，需要添加更多的检查，以保证不返回None，或者添加warn
            return_data = None
            last_data = self.data_queue_dict[frame_id].get()
            return_data = last_data  # 如果队列长度为1，则这个数据就是最近的数据
            while not self.data_queue_dict[frame_id].empty():
                data = self.data_queue_dict[frame_id].get()
                if data.header.stamp > current_time:
                    if data.header.stamp - current_time < current_time - last_data.header.stamp:
                        return_data = data
                    else:
                        return_data = last_data
                else:
                    last_data = data
            if return_data is None:
                rospy.logwarn("[  ROSDataset ] data_queue_dict[{}] is empty".format(frame_id))
                return None
            return return_data
        else:
            rospy.logwarn("[  ROSDataset ] data_queue_dict[{}] is empty".format(frame_id))
            return None

    def run(self):
        self.__init_all_subscriber()
        rospy.spin()

    def __init_all_subscriber(self):
        for key, value in self.topic_dict.items():
            msg_type = globals()[value.split("/")[-1]]
            self.subscriber_list.append(rospy.Subscriber(key, msg_type, self.__callback))

    def __callback(self, msg):
        frame_id = msg.header.frame_id
        if frame_id.startswith("/"):  # 修正frame_id
            frame_id = frame_id[1:]

        if frame_id not in self.frame_id_list:
            self.frame_id_list.append(frame_id)
            self.data_queue_dict[frame_id] = Queue(maxsize=self.queue_size)

        # 如果是离线数据，使用系统时间重置msg时间
        if self.offline:
            msg.header.stamp = rospy.Time.now()

        if self.data_queue_dict[frame_id].full():
            self.data_queue_dict[frame_id].get()
        self.data_queue_dict[frame_id].put(msg)


#
# class ROSDataset:
#     """ros数据获取接口
#     传入需要订阅的topic list,单独开启一个线程维护一个存储数据队列
#     """
#
#     def __init__(self, camera_topic_list, lidar_topic_list, queue_size=10, online=False):
#         self.camera_topic_list = camera_topic_list
#         self.lidar_topic_list = lidar_topic_list
#         self.QUENE_MAX_SIZE = queue_size
#         self.online = online
#
#         # for storing data
#         self.subscriber_list = []  # 订阅者列表
#         self.data_queue_dict = {}  # 数据队列字典,key为frame_id,value为队列
#
#         # start collect data thread
#         self.t = threading.Thread(target=self.__run)
#         self.t.start()
#
#         # completeness check
#         # 简单的进行完备性检查
#         time.sleep(1)
#
#         self.real_frame_id_list = self.__get_real_frame_id_list()
#         rospy.loginfo("[  ROSDataset ] real frame id list: {}".format(self.real_frame_id_list))
#
#     def __run(self):
#         self.subscriber_list_init()
#         rospy.spin()
#
#     def subscriber_list_init(self):
#         # 根据topic的类别选择不同的订阅消息类型
#         # init all camera subscribers
#         # NOTE : 采集数据均为compressed image, 这样数据量会小很多
#         if self.camera_topic_list:
#             for topic in self.camera_topic_list:
#                 self.subscriber_list.append(rospy.Subscriber(topic, CompressedImage, self.__callback))
#         # init all lidar subscribers
#         if self.lidar_topic_list:
#             for topic in self.lidar_topic_list:
#                 self.subscriber_list.append(rospy.Subscriber(topic, PointCloud2, self.__callback))
#
#     def __callback(self, msg):
#         frame_id = msg.header.frame_id
#         frame_id = correct_frame_id(frame_id)
#         if frame_id not in self.data_queue_dict:
#             self.data_queue_dict[frame_id] = Queue(maxsize=self.QUENE_MAX_SIZE)
#
#         # offline use system time reset msg time
#         if not self.online:
#             msg.header.stamp = rospy.Time.now()
#
#         if self.data_queue_dict[frame_id].full():
#             self.data_queue_dict[frame_id].get()
#         self.data_queue_dict[frame_id].put(msg)
#
#     def __get_the_closest_data(self, current_time, frame_id):
#         """获取时间最接近的数据"""
#         if not self.data_queue_dict[frame_id].empty():
#             # 遍历队列,找到时间最接近的数据
#             # 因为队列中的数据是按时间顺序排列的,所以只需要遍历一遍即可
#             # 最近的数据分几种存在位置
#             # - 如果队列长度为1,则这个数据就是最近的数据
#             # - 如果队列长度大于1,则需要遍历队列,找到时间最接近的数据
#             # - - 如果出现在队列中间位置，需要比较前后两个数据的时间差，找到最接近的数据
#             # - - 如果出现在队列开头位置，则返回队列第一个数据
#             # - - 如果出现在队列结尾位置，则返回队列最后一个数据
#             # - 如果队列长度为0,则返回None
#             # TODO ： 目前的检查不够完备，需要添加更多的检查，以保证不返回None，或者添加warn
#             return_data = None
#             last_data = self.data_queue_dict[frame_id].get()
#             return_data = last_data  # 如果队列长度为1，则这个数据就是最近的数据
#             while not self.data_queue_dict[frame_id].empty():
#                 data = self.data_queue_dict[frame_id].get()
#                 if data.header.stamp > current_time:
#                     if data.header.stamp - current_time < current_time - last_data.header.stamp:
#                         return_data = data
#                     else:
#                         return_data = last_data
#                 else:
#                     last_data = data
#             if return_data is None:
#                 rospy.logwarn("[  ROSDataset ] data_queue_dict[{}] is empty".format(frame_id))
#                 return None
#             return return_data
#         else:
#             rospy.logwarn("[  ROSDataset ] data_queue_dict[{}] is empty".format(frame_id))
#             return None
#
#     def __get_real_frame_id_list(self):
#         """获取所有frame_id"""
#         return list(self.data_queue_dict.keys())
#
#     def __all_frame_dict_check(self, all_frame_dict):
#         """检查是否所有的frame都有数据"""
#         if len(all_frame_dict) == len(self.real_frame_id_list):
#             for key, value in all_frame_dict.items():
#                 if value is None:
#                     return False
#         else:
#             return False
#         return True
#
#     def get_data(self, current_time, frame_id_list=None):
#         """获取与current_time最近的所有数据"""
#         if frame_id_list is None:
#             frame_id_list = self.real_frame_id_list
#         all_frame_dict = {}
#         for frame_id in frame_id_list:
#             all_frame_dict[frame_id] = self.__get_the_closest_data(current_time, frame_id)
#
#         if self.__all_frame_dict_check(all_frame_dict):
#             return all_frame_dict
#         else:
#             return None
