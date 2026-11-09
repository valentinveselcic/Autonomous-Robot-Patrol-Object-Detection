import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped

from cv_bridge import CvBridge
import cv2
import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class YellowObjectDetector(Node):
    def __init__(self):
        super().__init__('yellow_object_detector')

        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/oakd/rgb/preview/camera_info',
            self.camera_info_callback,
            10
        )

        self.rgb_sub = Subscriber(self, Image, '/oakd/rgb/preview/image_raw')
        self.depth_sub = Subscriber(self, Image, '/oakd/rgb/preview/depth')

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.synced_callback)

        qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        )
        self.point_pub = self.create_publisher(
            Point,
            '/yellow_object_point',
            qos
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id

    def synced_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.fx is None:
            self.get_logger().warn('CameraInfo still not received, skipping frame.')
            return

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([35, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        best_label = -1
        best_area = 0
        min_area = 200  

        for i in range(1, num_labels):  # 0 je pozadina
            x, y, w, h, area = stats[i]
            if area > min_area and area > best_area:
                best_area = area
                best_label = i

        if best_label == -1:
  
            return

        cx_img, cy_img = centroids[best_label]
        u = int(round(cx_img))
        v = int(round(cy_img))

        if v < 0 or v >= depth_m.shape[0] or u < 0 or u >= depth_m.shape[1]:
            return

        z = depth_m[v, u]
        if z <= 0 or np.isnan(z):
            return

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy
        z_cam = z

        point_cam = PointStamped()
        point_cam.header.stamp = rgb_msg.header.stamp
        point_cam.header.frame_id = self.camera_frame or rgb_msg.header.frame_id
        point_cam.point.x = float(x_cam)
        point_cam.point.y = float(y_cam)
        point_cam.point.z = float(z_cam)

        # Transformacija u map frame pomoću TF-a
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',                        # cilj
                point_cam.header.frame_id,    # izvor (kamera)
                rclpy.time.Time()             # zadnja dostupna transformacija
            )
        except Exception as e:
            self.get_logger().warn(f'Cant transform to map frame: {e}')
            return

        point_map_stamped = do_transform_point(point_cam, transform)

        out = Point()
        out.x = point_map_stamped.point.x
        out.y = point_map_stamped.point.y
        out.z = point_map_stamped.point.z

        self.point_pub.publish(out)



def main(args=None):
    rclpy.init(args=args)
    node = YellowObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
