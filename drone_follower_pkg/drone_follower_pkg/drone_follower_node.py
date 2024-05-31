import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandLong, CommandTOL, CommandBool


# STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
STATE_QOS = QoSProfile(depth=5, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

TARGET_QOS = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class DroneFollowerNode(Node):
    def __init__(self):
        super().__init__("drone_follower_node")

        self.in_air = False

        self.drone_state_messages = []
        # create service clients
        # for long command (data stream requests)...
        self.cmd_cli = self.create_client(CommandLong, "/mavros/cmd/command")
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("command service not available, waiting again...")

        # for mode changes ...
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("set_mode service not available, waiting again...")

        # for arming ...
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("arming service not available, waiting again...")

        # for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("takeoff service not available, waiting again...")

        # publisher for setpoint messages that are used to move the drone
        self.target_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        # sub for vehicle state
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_callback, STATE_QOS
        )

        # sub for pose target.
        self.pose_target_sub = self.create_subscription(
            PoseStamped, "/drone/next_drone_pose", self.pose_target_callback, TARGET_QOS
        )

        self.setup_for_flight()
        self.takeoff(target_alt=1.5)
        self.in_air = True  # wait for takeoff to finish to send movement commands

    def pose_target_callback(self, msg: PoseStamped):
        if self.in_air:
            self.get_logger().info(
                f"moving to {msg.pose.position} with {msg.pose.orientation}"
            )
            self.target_pub.publish(msg)

    def state_callback(self, msg: State):
        self.drone_state_messages.append(msg)
        self.get_logger().debug(f"Drone State. mode: {msg.mode} armed: {msg.armed}")

    def setup_for_flight(self, timeout=30, wait_for_standby=True, tries=3):
        """Prepares the drone for flight. returns True if process is successful returns False otherwise.

        Args:
            timeout: seconds wait at each step until timeout. default 30.
            wait_for_standby: whether to wait for standby or not. Defaults to 30.
            tries: How many times the function tries to setup.
        """

        # request all needed messages
        self.request_needed()

        for i in range(tries):
            if wait_for_standby:
                # wait until system status becomes standby
                for _ in range(timeout):
                    self.wait_for_new_state_message()
                    if self.drone_state_messages[-1].system_status == 3:
                        self.get_logger().info("System status: Standby")
                        break

            # change mode to GUIDED
            self.change_mode("GUIDED")
            self.get_logger().info("Mode set to GUIDED")

            # try to arm the drone.
            for _ in range(timeout):
                # send an arm request
                self.arm_request()
                self.get_logger().info("Arming request sent.")

                # wait for a new state message
                self.wait_for_new_state_message()
                if self.drone_state_messages[-1].armed:
                    self.get_logger().info("Arming successful")
                    break

            # check if the drone is ready or not
            self.wait_for_new_state_message(timeout)

            last_state = self.drone_state_messages[-1]

            if last_state.armed and last_state.guided:
                self.get_logger().info("Drone is ready for flight")
                return True

            self.get_logger().info(
                f"Drone Setup failed armed:{last_state.armed} guided:{last_state.guided} mode:{last_state.mode}"
            )

        return False

    def wait_for_new_state_message(self, timeout=30):
        """
        Wait for new state message to be received.  These are sent at
        1Hz so calling this is roughly equivalent to one second delay.

        Args:
            timeout: Seconds to wait until timeout.

        """
        start_len = len(self.drone_state_messages)
        for _ in range(timeout):
            rclpy.spin_once(self)
            if len(self.drone_state_messages) > start_len:
                break

    def request_needed(self, msg_interval=100000):
        """Requests all needed messages from the fcu.
            uses request_data_stream function.

        Args:
            msg_interval: message send interval in microseconds. applies to all data streams called by the function.
        """

        # local position
        self.request_data_stream(32, msg_interval)
        # global position
        self.request_data_stream(33, msg_interval)
        # gps raw
        self.request_data_stream(24, msg_interval)
        # raw imu
        self.request_data_stream(27, msg_interval)

    def request_data_stream(self, msg_id, msg_interval):
        """Request data stream from the vehicle. Otherwise in some cases (e.g. when using ardupilot sitl), the vehicle will not send any data to mavros.

        message ids: https://mavlink.io/en/messages/common.html

        Args:
            msg_id: MAVLink message ID
            msg_interval: interval in microseconds
        """

        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        # wait for response
        rclpy.spin_until_future_complete(self, future)

    def change_mode(self, new_mode):
        """Changes the mode of the drone.

        Args:
            new_mode: target mode.
        """
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)

    def wait_for(self, seconds):
        """
        Wait for the given number of seconds.
        """
        self.get_logger().info("Waiting for {} seconds".format(seconds))
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < seconds:
            rclpy.spin_once(self)
        return True

    def arm_request(self):
        """Send an arm request to the drone"""
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)

    def takeoff(self, target_alt):
        """Takeoff to the target altitude

        Args:
            target_alt: target altitude.
        """
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = float(target_alt)
        future = self.takeoff_cli.call_async(takeoff_req)
        return rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)
    drone_follower_node = DroneFollowerNode()
    rclpy.spin(drone_follower_node)
    drone_follower_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
