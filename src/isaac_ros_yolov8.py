import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from isaac_ros_yolov8.yolov8 import YoloV8
import cv2
import numpy as np

class IsaacSimCameraBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_yolov8_node')


        self.create_subscription(
            Image,
            '/sim/camera/rgb',
            self.on_image_received,
            10
        )


        self.model = YoloV8(model_path='yolov8n.onnx')

    def on_image_received(self, msg):

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape(msg.height, msg.width, 3)

        # Run YOLOv8 inference
        outputs = self.model.infer(frame)


        for det in outputs:
            x1, y1, x2, y2, conf, cls = det
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)


        cv2.imshow("YOLOv8 Isaac Sim", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = IsaacSimCameraBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
