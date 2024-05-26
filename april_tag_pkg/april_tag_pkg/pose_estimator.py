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
import numpy as np

SUB_QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
)

PUB_QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
)

POSE_QOS = rclpy.qos.QoSProfile(
    depth=10,
    durability=rclpy.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=rclpy.QoSReliabilityPolicy.RELIABLE,
)


class PoseEstimator(Node):
    def __init__(self):
        super().__init__("pose_estimator_node")

        self.last_apriltag_detection_msg = None
        self.last_drone_pose_msg = None
        self.last_follow_target_pose_msg = None

        self.declare_parameter(
            name="pose_estimator_base_link_frame",
            value="base_link",
            descriptor="Frame the pose estimator uses as the base frame. All pose data will be converted to this frame.",
        )

        self.declare_parameter(
            name="drone_follow_altitude",
            value=3.0,
            descriptor="Altitude of the drone in meters while following the target.",
        )

        self.declare_parameter(
            name="drone_next_pose_pub_rate",
            value=10.0,
            descriptor="Publish rate of the drones next pose message.",
        )

        self.drone_next_pose_pub_rate: float = (
            self.get_parameter(name="drone_next_pose_pub_rate")
            .get_parameter_value()
            .double_value
        )

        self.drone_alt_param: float = (
            self.get_parameter(name="drone_follow_altitude")
            .get_parameter_value()
            .double_value
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

        # sub for local position of the target that will be followed. Unused for now.
        self.follow_target_pos_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic="/drone/follow_target_pose",
            callback=self.follow_target_pose_callback,
            qos_profile=SUB_QOS,
        )

        # publisher for the next position of the drone calculated by this node.
        # This is listened by drone_follower to give movement commands to the drone
        self.drone_next_pos_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic="/drone/next_drone_pose",
            qos_profile=POSE_QOS,
        )

        # tf2 buffer and listener.
        # We need this because apriltag detections are in camera_frame so we need to convert them to base_link frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)

        self.drone_alt_param_timer = self.create_timer(
            timer_period_sec=0.2, callback=self.get_drone_alt_param
        )

        self.drone_next_pose_timer = self.create_timer(
            timer_period_sec=1.0 / self.drone_next_pose_pub_rate,
            callback=self.send_drone_next_pose,
        )

    def follow_target_pose_callback(self, msg: PoseStamped):
        self.last_follow_target_pose_msg = msg

    def drone_pose_callback(self, msg: PoseStamped):
        self.last_drone_pose_msg = msg

    def apriltag_detection_callback(self, msg: AprilTagDetectionArray):
        self.last_apriltag_detection_msg = msg

    def get_drone_alt_param(self):
        self.drone_alt_param = (
            self.get_parameter(name="drone_follow_altitude")
            .get_parameter_value()
            .double_value
        )

    def send_drone_next_pose(self):
        """Publish drones next pose for drone_follower_node. NOTE: Right now only uses pose info obtained from apriltags."""
        # check if drones pose and apriltag pose messages are received.
        if not self.last_drone_pose_msg and self.last_apriltag_detection_msg:
            return

        # transform all data to base_link_frame
        apriltag_poses = self.msg_to_base_link(
            self.last_apriltag_detection_msg, self.base_link_frame
        )

        # TODO: fuse all apriltag poses here instead of using the first one.

        to_send = self.calculate_next_pose(
            Pose3D.from_ros_message(self.last_drone_pose_msg),
            follow_target_pose=apriltag_poses[0],
        )

        msg_to_send = PoseStamped()
        msg_to_send.header.frame_id = self.base_link_frame
        msg_to_send.header.stamp = self.get_clock().now().to_msg()
        (
            msg_to_send.pose.position.x,
            msg_to_send.pose.position.y,
            msg_to_send.pose.position.z,
        ) = to_send.get_position()
        (
            msg_to_send.pose.orientation.x,
            msg_to_send.pose.orientation.y,
            msg_to_send.pose.orientation.z,
            msg_to_send.pose.orientation.w,
        ) = to_send.get_quaternion()

        self.drone_next_pos_pub.publish(msg_to_send)

    def msg_to_base_link(
        self, msg, target_frame: str, source_frame: str = None
    ) -> Union[List[Pose3D], Pose3D, None]:
        if not source_frame:
            source_frame = msg.header.frame_id

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

    def calculate_next_pose(
        self,
        drone_pose: Pose3D,
        follow_target_pose: Pose3D,
        displacement: np.ndarray = None,
    ) -> Pose3D:
        """Calculates the next pose of the drone using pose of the follow target relative to the drone.

        Args:
            drone_pose (Pose3D): Current pose of the drone.
            follow_target_pose (Pose3D): Pose of the follow target relative to the drone.
            displacement (np.ndarray): 3 dim numpy array that will be added to the final estimate. NOTE: independent of the drone follow altitude.

        Returns:
            drone_next_pose: Drones next pose.
        """

        # check if all data belongs to the same frame.
        if drone_pose.get_frame() != follow_target_pose.get_frame():
            raise ValueError()

        position = (
            drone_pose.get_position()
            + follow_target_pose.get_position()
            + np.array([0.0, 0.0, self.drone_alt_param])
            + displacement
        )
        orientation = follow_target_pose.get_quaternion()

        return Pose3D(
            position=position,
            quaternion=orientation,
            frame=drone_pose.get_frame(),
            time=self.get_clock().now(),
        )


def main(args=None):
    rclpy.init(args=args)
    pose_estimator_node = PoseEstimator()
    rclpy.spin(pose_estimator_node)
    pose_estimator_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
