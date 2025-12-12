import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/sim/camera/rgb',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  
        
        self.get_logger().info('YOLOv8 Node Started - Subscribing to /sim/camera/rgb')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')        
            results = self.model.predict(cv_image, device='cpu', verbose=False)            
            annotated_frame = results[0].plot()            
            cv2.imshow('YOLOv8 Detections', annotated_frame)
            cv2.waitKey(1)          
            for box in results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls]
                self.get_logger().info(f'Detected: {label} ({conf:.2f})')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
