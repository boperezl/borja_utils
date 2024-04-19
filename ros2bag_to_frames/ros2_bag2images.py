import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
import sys 

class ImageToVideoConverter(Node):
    def __init__(self, topic, frame_path): 
        super().__init__('image_to_video_converter')
        self.bridge = cv_bridge.CvBridge()
        self.frame = 0
        self.frame_path = frame_path
        self.subscription = self.create_subscription(Image, topic, self.image_callback, 10 ) #/albert/color/image_raw

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            #print(f"WRITING img {self.frame_path}frame_{self.frame}.png")
            cv2.imwrite(f"{self.frame_path}frame_{self.frame}.png", cv_image)
            self.frame+=1
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

def main(args=None):
    if args is None:
        args = sys.argv
    
    if len(args) != 3:
        print("Usage: python ros2_bag2images.py <topic> <frames_path>")
        return
    
    rclpy.init()
    image_to_video_converter = ImageToVideoConverter(args[1], args[2])

    rclpy.spin(image_to_video_converter)
    image_to_video_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()