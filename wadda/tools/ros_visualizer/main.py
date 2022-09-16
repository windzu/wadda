import random
from argparse import ArgumentParser

# ros
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from visualization_msgs.msg import ImageMarker, Marker, MarkerArray
from foxglove_msgs.msg import ImageMarkerArray


class ROSVisualizer(object):
    COLOR_LIST = [
        ColorRGBA(1.0, 0.0, 0.0, 0.8),  # red
        ColorRGBA(0.0, 1.0, 0.0, 0.8),  # green
        ColorRGBA(0.0, 0.0, 1.0, 0.8),  # blue
        ColorRGBA(1.0, 1.0, 0.0, 0.8),  # yellow
        ColorRGBA(1.0, 0.0, 1.0, 0.8),  # purple
        ColorRGBA(1.0, 0.5, 0.0, 0.8),  # orange
    ]

    def __init__(self):

        # 初始化工作
        ## - 初始化全局颜色显示dict,用于显示不同类别的检测结果
        self.color_dict = {}

        # 1. 获取所有已经发布的topic
        rospy.sleep(1)
        self.all_topics = rospy.get_published_topics()

        # 2. 根据功能需求的不同，从所有的topic中筛选出需要的topic(通过消息类型来筛选)
        ## - 筛选检测结果的topic
        self.detected_objects_topic_dict = self.__filter_detected_objects_topic()

        # 3. 订阅筛选出来的topic
        ## - 订阅检测结果的topic
        self.detected_objects_sub_dict = self.__subscribe_detected_objects_topic()

        # 4. 创建一个publisher dict,用于发布检测结果
        self.pub_dict = {}

        rospy.loginfo("[ ROSVisualizer ] init success , waiting for data...")

    def __filter_detected_objects_topic(self):
        """筛选检测结果的topic
        目前检测结果的消息类型均为 autoware_msgs/DetectedObjectArray
        Returns:
            dict: key为topic的名称,value为topic的消息类型
        """
        self.detected_objects_topic_dict = {}
        for topic_name, topic_type in self.all_topics:
            if topic_type == "autoware_msgs/DetectedObjectArray":
                self.detected_objects_topic_dict[topic_name] = topic_type
        return self.detected_objects_topic_dict

    def __subscribe_detected_objects_topic(self):
        """订阅检测结果的topic
        默认消息类型均为 autoware_msgs/DetectedObjectArray
        """
        detected_objects_sub_dict = {}
        for topic_name, topic_type in self.detected_objects_topic_dict.items():
            detected_objects_sub_dict[topic_name] = rospy.Subscriber(
                topic_name, DetectedObjectArray, self.__detected_objects_callback
            )
        return detected_objects_sub_dict

    def __detected_objects_callback(self, msg):
        """检测结果的回调函数
        将autoware_msgs/DetectedObjectArray转换为marker,分以下几种情况:
        - 检测结果为3d bbox,则将3d bbox转换为 MarkerArray
        - 检测结果为2d bbox,则将2d bbox转换为 ImageMarker
        """
        assert isinstance(msg, DetectedObjectArray)  # 确保消息类型正确
        if msg.objects is None or len(msg.objects) == 0:
            return
        if msg.objects[0].width > 0:  # width > 0,说明检测结果为2d bbox
            result = self.__detected_objects_2d_callback(msg)
        else:
            result = self.__detected_objects_3d_callback(msg)

        # 发布的topic通过frame_id构成: /frame_id/marker
        pub_topic = msg.objects[0].header.frame_id + "/marker"
        if pub_topic not in self.pub_dict:
            self.pub_dict[pub_topic] = rospy.Publisher(pub_topic, type(result), queue_size=1)
        self.pub_dict[pub_topic].publish(result)

        rospy.loginfo("[ ROSVisualizer ] publish topic: {}".format(pub_topic))

    def __detected_objects_3d_callback(self, msg):
        """检测结果为3d bbox的回调函数"""

        def create_cube_marker(object, id):
            bbox3d_marker = Marker()
            bbox3d_marker.header = object.header
            bbox3d_marker.header.stamp = rospy.Time.now()
            bbox3d_marker.type = Marker.CUBE
            bbox3d_marker.action = Marker.ADD
            bbox3d_marker.ns = "CUBE"
            bbox3d_marker.id = id

            bbox3d_marker.pose = object.pose
            bbox3d_marker.scale = object.dimensions

            # set color
            if object.label not in self.color_dict:
                self.color_dict[object.label] = self.__random_color()
            bbox3d_marker.color = self.color_dict[object.label]

            bbox3d_marker.lifetime = rospy.Duration(0.1)
            return bbox3d_marker

        def create_arrow_marker(object, id):
            """创建一个箭头,用于显示方向"""
            bbox3d_marker = Marker()
            bbox3d_marker.header = object.header
            bbox3d_marker.header.stamp = rospy.Time.now()
            bbox3d_marker.type = Marker.ARROW
            bbox3d_marker.action = Marker.ADD
            bbox3d_marker.ns = "ARROW"
            bbox3d_marker.id = id

            bbox3d_marker.pose = object.pose
            bbox3d_marker.scale.x = object.dimensions.x
            bbox3d_marker.scale.y = 1
            bbox3d_marker.scale.z = 1
            bbox3d_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # red arrow
            bbox3d_marker.lifetime = rospy.Duration(0.1)
            return bbox3d_marker

        if msg is None or not isinstance(msg, DetectedObjectArray):
            return None

        marker_array = MarkerArray()
        id = 0
        for object in msg.objects:
            # cube marker
            cube_marker = create_cube_marker(object, id)
            marker_array.markers.append(cube_marker)
            id += 1
            # arrow marker
            arrow_marker = create_arrow_marker(object, id)
            marker_array.markers.append(arrow_marker)
            id += 1
        return marker_array

    def __detected_objects_2d_callback(self, msg):
        """检测结果为2d bbox的回调函数"""

        def create_rect_marker(object, id):
            """通过ImageMarker创建一个矩形
            因为ImageMarker其中的type本不支持rect,
            所以通过LINE_LIST来模拟一个rect
            """
            image_marker = ImageMarker()
            image_marker.id = id
            image_marker.ns = "autoware_detected_object"  # 需要调整
            image_marker.header = object.header
            image_marker.header.stamp = rospy.Time.now()
            image_marker.type = ImageMarker.LINE_LIST
            image_marker.action = ImageMarker.ADD
            image_marker.scale = 2
            image_marker.points = []
            image_marker.outline_colors = []
            image_marker.fill_color = ColorRGBA(0.0, 0.0, 1.0, 0.8)

            # calculate the 4 points of the rect
            x = object.x
            y = object.y
            width = object.width
            height = object.height

            image_marker.points.append(Point(x, y, 0))
            image_marker.points.append(Point(x + width, y, 0))
            image_marker.points.append(Point(x + width, y, 0))
            image_marker.points.append(Point(x + width, y + height, 0))
            image_marker.points.append(Point(x + width, y + height, 0))
            image_marker.points.append(Point(x, y + height, 0))
            image_marker.points.append(Point(x, y + height, 0))
            image_marker.points.append(Point(x, y, 0))

            # set the color of the rect
            if object.label not in self.color_dict:
                self.color_dict[object.label] = self.__random_color()
            for i in range(8):
                image_marker.outline_colors.append(self.color_dict[object.label])

            return image_marker

        if msg is None or not isinstance(msg, DetectedObjectArray):
            return None

        marker_array = ImageMarkerArray()
        id = 0
        for object in msg.objects:
            # rect marker
            rect_marker = create_rect_marker(object, id)
            marker_array.markers.append(rect_marker)
            id += 1
        return marker_array

    def __random_color(self):
        """随机生成一个颜色
        最好从一些预设的鲜艳颜色中随机选取
        """
        if len(self.COLOR_LIST) > 0:
            color_index = random.randint(0, len(self.COLOR_LIST) - 1)
            color = self.COLOR_LIST[color_index]
            self.COLOR_LIST.pop(color_index)
        else:
            color = ColorRGBA(random.random(), random.random(), random.random(), 0.8)
        return color


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("--path", type=str, help="data root path")
    parser.add_argument("--pro", action="store_true", help="whether to enable professional mode")
    return parser.parse_args()


def main(args=None):
    rospy.init_node("ros_vis_tool", anonymous=True)
    ros_visualizer = ROSVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
