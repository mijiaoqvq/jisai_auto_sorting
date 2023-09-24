import logging
import torch
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from interfaces.msg import ItemInfo


class Detect(Node):

    def __init__(self):
        super().__init__('detect')
        self.bridge = CvBridge()
        self.declare_parameter('model_path', 'best.pt')
        self.subscription = self.create_subscription(
            Image,
            '/arm_camera/img_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            ItemInfo,
            'item_info',
            10)

        path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info('Waiting for model initializing...')
        self.model = YOLO(path)
        self.get_logger().info('Model initialized successful!')

    def listener_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)
        result = results[0]
        boxes = result.boxes

        if boxes.cls.shape[0] > 0:
            info = ItemInfo()
            info.x = float((boxes.xyxy[0, 0] + boxes.xyxy[0, 2]) / 2)
            info.y = float((boxes.xyxy[0, 1] + boxes.xyxy[0, 3]) / 2)
            info.id = int(boxes.cls[0])
            self.publisher.publish(info)


def main(args=None):
    logging.disable(logging.CRITICAL)
    rclpy.init(args=args)
    detect = Detect()
    rclpy.spin(detect)
    detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
