import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from scipy.spatial.transform import Rotation as R
from typing import Union, List


class Pose3D:
    def __init__(
        self, position: list[float], quaternion: list[float], frame: str, time: float
    ):
        """Initialize Pose3D with position, quaternion rotation, frame, and time."""
        self.position = np.array(position)  # (x, y, z) coordinates
        self.quaternion = np.array(quaternion)  # (x, y, z, w) quaternion
        self.frame = frame  # Frame ID
        self.time = time  # Time in seconds

    def get_position(self) -> np.ndarray:
        """Return the position as a numpy array."""
        return self.position

    def set_position(self, position: list[float]) -> None:
        """Set the position using a list of floats."""
        self.position = np.array(position)

    def get_quaternion(self) -> np.ndarray:
        """Return the quaternion as a numpy array."""
        return self.quaternion

    def set_quaternion(self, quaternion: list[float]) -> None:
        """Set the quaternion using a list of floats."""
        self.quaternion = np.array(quaternion)

    def get_frame(self) -> str:
        """Return the frame ID as a string."""
        return self.frame

    def set_frame(self, frame: str) -> None:
        """Set the frame ID."""
        self.frame = frame

    def get_time(self) -> float:
        """Return the time as a float."""
        return self.time

    def set_time(self, time: float) -> None:
        """Set the time."""
        self.time = time

    def distance_to(self, other_pose: "Pose3D") -> float:
        """Calculate the Euclidean distance to another Pose3D."""
        if not isinstance(other_pose, Pose3D):
            raise ValueError("Argument must be an instance of Pose3D")
        return np.linalg.norm(self.position - other_pose.position)

    @classmethod
    def from_ros_message(
        cls, msg: Union[PoseStamped, PoseWithCovarianceStamped]
    ) -> "Pose3D":
        """Create a Pose3D instance from a ROS2 PoseStamped message."""
        if not isinstance(msg, [PoseStamped, PoseWithCovarianceStamped, Pose]):
            raise ValueError(
                "Message must be an instance of PoseStamped, PoseWithCovarianceStamped or Pose"
            )

        if isinstance(msg, Pose):
            position = msg.position
            orientation = msg.orientation
        else:
            position = msg.pose.position
            orientation = msg.pose.orientation

        position = [position.x, position.y, position.z]

        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        frame = msg.header.frame_id
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        return cls(position, quaternion, frame, time)

    @classmethod
    def solve_quat(
        cls, quat_initial: np.ndarray, quat_target: np.ndarray
    ) -> np.ndarray:
        """Finds the quaternion that rotates one orientation to another.

        Args:
            quat_initial (_type_): _description_
            quat_target (_type_): _description_

        Returns:
            rotation_quaternion: Quaternion that when applied to initial quaternion makes it equal to quaternion target.
        """
        return (R.from_quat(quat_target) * R.from_quat(quat_initial).inv()).as_quat()
