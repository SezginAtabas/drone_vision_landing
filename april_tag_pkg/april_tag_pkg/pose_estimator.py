import rclpy
from rclpy.node import Node

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

from message_filters import ApproximateTimeSynchronizer, Subscriber

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose_with_covariance_stamped

import numpy as np

POSE_QOS = rclpy.qos.QoSProfile(
    depth=10,
    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
)

LOCAL_QOS = rclpy.qos.QoSProfile(
    depth=10,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Ensure reliability matches MAVROS publisher
)


class PoseEstimator(Node):
    def __init__(self):
        super().__init__("pose_estimator_node")
        self.initialize_parameters()

        # Listen to tf2 for camera_frame to base_link_frame transform.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)
        self.cam_to_base_transform = None
        self.transform_timer = self.create_timer(0.5, self.get_transform)

        self.drone_next_pos_pub = self.create_publisher(
            PoseStamped, "/drone/next_drone_pose", qos_profile=POSE_QOS
        )

        # We need to sync apriltag messages with drone local position messages.
        # NOTE: Current slop in ApproximateTimeSynchronizer is for testing could be lower.
        apriltag_sub = Subscriber(self, AprilTagDetectionArray, "/tag_detections")
        drone_sub = Subscriber(
            self,
            PoseStamped,
            "/mavros/local_position/pose",
            qos_profile=LOCAL_QOS,
        )
        ats = ApproximateTimeSynchronizer([apriltag_sub, drone_sub], 10, 0.05)
        ats.registerCallback(self.next_pose_callback)

    def initialize_parameters(self):
        """
        The `initialize_parameters` function initializes parameters for a drone's follow behavior, including
        base link frame, camera frame, and drone follow altitude.
        """

        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("camera_frame", "zed_camera_link")
        self.declare_parameter("drone_follow_altitude", 3.0)

        self.drone_follow_displacement = np.array([0.0, 0.0, 0.0])
        self.drone_alt_param = (
            self.get_parameter("drone_follow_altitude")
            .get_parameter_value()
            .double_value
        )
        self.base_link_frame = (
            self.get_parameter("base_link_frame").get_parameter_value().string_value
        )
        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

    def get_transform(self):
        """
        This function attempts to retrieve a transform between two frames using a transform buffer,
        with error handling in case the transform lookup fails.
        """
        if not self.cam_to_base_transform:
            try:
                self.cam_to_base_transform = self.tf_buffer.lookup_transform(
                    self.base_link_frame, self.camera_frame, rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().info(f"Failed to get transform: {e}")
        else:
            self.transform_timer.destroy()

    def next_pose_callback(self, apriltag_msg, drone_pose):
        """
        The `next_pose_callback` function processes AprilTag detections and publishes the next position for
        the drone to follow.

        :param apriltag_msg: `apriltag_msg` is a message containing detections of AprilTags. It has
        a field `detections` which is a list of detected AprilTags along with their poses
        :param drone_pose: `drone_pose` is a parameter representing the pose of the drone. It is used in the
        `next_pose_callback` function to set the orientation of the `msg_to_pub` PoseStamped message before
        publishing it
        :return: The function will return `None` implicitly if the conditions for the early returns are met.

        """
        if not self.cam_to_base_transform:
            self.get_logger().info("Waiting for transform.")
            return

        # NOTE:We need this because apriltag detection node always sends messages even if no apriltags are detected.
        if len(apriltag_msg.detections) < 1:
            return
        
        # We only use position data from the apriltags and keep orientation same.
        tag_poses = [
            do_transform_pose_with_covariance_stamped(
                det.pose, self.cam_to_base_transform
            ).pose.pose
            for det in apriltag_msg.detections
        ]
        tag_positions = np.array(
            [[pose.position.x, pose.position.y, pose.position.z] for pose in tag_poses]
        )
        tag_mean_position = (
            np.mean(tag_positions, axis=0) + self.drone_follow_displacement
        )

        msg_to_pub = PoseStamped()
        msg_to_pub.header.stamp = self.get_clock().now().to_msg()
        msg_to_pub.header.frame_id = self.base_link_frame
        (
            msg_to_pub.pose.position.x,
            msg_to_pub.pose.position.y,
            msg_to_pub.pose.position.z,
        ) = tag_mean_position
        msg_to_pub.pose.orientation = drone_pose.pose.orientation

        self.drone_next_pos_pub.publish(msg_to_pub)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
