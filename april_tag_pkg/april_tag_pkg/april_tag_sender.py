import rclpy
import rclpy.qos
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import cv2

MY_QOS = rclpy.qos.QoSProfile(
    depth=10,
    reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
    durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
)


class AprilTagSender(Node):
    def __init__(self):
        super().__init__("april_tag_sender")

        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image,
            "/zed/zed_node/left/image_rect_color",
            self.img_callback,
            MY_QOS,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/zed/zed_node/left/camera_info",
            self.camera_info_callback,
            MY_QOS,
        )

        self.img_pub = self.create_publisher(Image, "/image", MY_QOS)

        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera_info", MY_QOS)

        # we need to convert zed images to bgr8 format before sending them.

        
    def img_callback(self, msg: Image):
        try:
            # Convert the ROS Image message to a CV2 Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            bgr_image = cv_image[:, :, :3]
            
            # Convert to BGR8 format
            #bgr8_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
            
            # Convert back to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
            ros_image.header = msg.header
            self.img_pub.publish(ros_image)
            self.get_logger().info("converted image")

        except Exception as e:
            self.get_logger().error(f"CvBridgeError: {e}")

        

    def camera_info_callback(self, msg):
        self.camera_info_pub.publish(msg)


def main():
    rclpy.init()
    april_tag_sender = AprilTagSender()
    rclpy.spin(april_tag_sender)
    april_tag_sender.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
