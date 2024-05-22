import rclpy
import rclpy.qos
from rclpy.node import Node

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from typing import List, Union

from pypose3D import Pose3D


SUB_QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
)

PUB_QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
)


class PoseEstimator(Node):
    def __init__(self):
        super().__init__("pose_estimator_node")

        self.declare_parameter(
            name="pose_estimator_base_link_frame",
            value="base_link",
            descriptor="Frame the pose estimator uses as the base frame. All pose data will be converted to this frame.",
        )

        self.base_link_frame: str = (
            self.get_parameter(name="pose_estimator_base_link_frame")
            .get_parameter_value()
            .string_value
        )

        # sub to the apriltag detections published by the detector
        self.apriltag_detection_sub = self.create_subscription(
            msg_type=AprilTagDetectionArray,
            topic="/tag_detections",
            callback=self.apriltag_detection_callback,
            qos_profile=SUB_QOS,
        )

        # sub for local position of the drone.
        self.drone_pos_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic="/mavros/local_position/pose",
            callback=self.drone_pose_callback,
            qos_profile=SUB_QOS,
        )

        # sub for local position of the target that will be followed.
        self.drone_pos_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic="/drone/follow_target_pose",
            callback=self.follow_target_pose_callback,
            qos_profile=SUB_QOS,
        )

        # publisher for the next position of the drone calculated by this node.
        # This is listened by drone_follower to give movement commands to the drone
        self.drone_target_pos_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic="/drone/next_drone_pose",
            qos_profile=PUB_QOS,
        )

        # tf2 buffer and listener.
        # We need this because apriltag detections are in camera_frame so we need to convert them to base_link frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)

    def msg_to_base_link(
        self, msg, target_frame: str, source_frame: str = None
    ) -> Union[List[Pose3D], Pose3D, None]:
        if not source_frame:
            source_frame = msg.header

            # Get the transform between the source_frame and the target_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=self.get_clock().now(),
            )

        except Exception as e:
            self.get_logger().warn(message=e)
            return

        # if msg is AprilTagDetectionArray transform all tag poses.
        # NOTE: For now we are ignoring pixel coordinates of the tags.
        if isinstance(msg, AprilTagDetectionArray):
            transformed_poses = []
            for apriltag in msg.detections:
                transformed_poses.append(
                    Pose3D.from_ros_message(
                        do_transform_pose(pose=apriltag.pose, transform=transform)
                    )
                )
            return transformed_poses

        return Pose3D.from_ros_message(
            do_transform_pose(pose=msg.pose, transform=transform)
        )


def main(args=None):
    rclpy.init(args=args)
    pose_estimator_node = PoseEstimator()
    rclpy.spin(pose_estimator_node)
    pose_estimator_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
