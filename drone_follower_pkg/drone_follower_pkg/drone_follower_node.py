import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import (
    SetMode,
    CommandLong,
    CommandTOL,
    CommandBool,
    MessageInterval,
)

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

        #  <------ initialize parameters ------>

        # flight duration of the drone in seconds.
        # drone will start the landing sequence when this value is exceeded
        self.declare_parameter(
            name="flight_duration",
            value=60,
        ).get_parameter_value().integer_value

        self.drone_flight_duration = Time(
            seconds=self.get_parameter("flight_duration")
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info(
            "Drone will fly for %d seconds before landing" % self.drone_flight_duration
        )

        # <------ Publishers and Subscribers ------>

        # publisher for set-point messages that are used to move the drone
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

        # <------ Create clients ------>

        # for long command (data stream requests)...
        self.cmd_cli = self.create_client(CommandLong, "/mavros/cmd/command")
        # for mode changes ...
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        # for arming ...
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        # for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        # to set interval between received MavLink messages
        self.message_interval_cli = self.create_client(
            MessageInterval, "/mavros/set_message_interval"
        )

        # Create some variables to keep track of the drone.
        self.in_air = False
        self.should_land = False
        self.drone_state_messages = []
        self.in_air_start_time = None
        self.ready_for_takeoff = False

        # MavLink messages to request from the drone flight controller.
        # These are drone position, attitude etc. And are requested using
        # set_all_message_interval function which makes async calls to
        # /mavros/set_message_interval service.
        # Message id, Interval in microseconds
        self.messages_to_request = (
            (32, 100000),  # local position
            (33, 100000),  # global position
            (24, 100000),  # gps raw
            (27, 100000),  # raw imu
        )

        # create timer to check flight duration every second.
        self.create_timer(1.0, self.check_flight_duration)

    def pose_target_callback(self, msg: PoseStamped):
        """
        Calculate the next pose of the drone using apriltag poses.

        Args:
            msg: Mean Pose of the detected apriltags relative to the drone. This pose data is in 'base_link' frame.
        """

        if self.should_land:
            return

        msg.pose.position.x += 0.0
        msg.pose.position.y += 0.0
        # drone attitude to follow the target with in meters
        msg.pose.position.z += 3.0

        if self.in_air:
            self.get_logger().info(
                f"moving to {msg.pose.position} with {msg.pose.orientation}"
            )
            self.target_pub.publish(msg)

    def check_flight_duration(self):
        """
        Check if the flight duration has elapsed. If so, send a land command.
        """

        if not self.in_air or self.in_air_start_time is None:
            return

        if self.get_clock().now() - self.in_air_start_time > self.drone_flight_duration:
            self.get_logger().info("Flight duration elapsed. Sending land command.")
            self.should_land = True
            self.land()
            return

        self.get_logger().info(
            f"Flight duration remaining: {self.drone_flight_duration} seconds."
        )

    def land(self):
        """
        Changes mode to land.
        """
        self.change_mode("LAND")

    def state_callback(self, msg: State):
        self.drone_state_messages.append(msg)
        self.get_logger().debug(f"Drone State. mode: {msg.mode} armed: {msg.armed}")

    def setup_for_flight(self, timeout=30, wait_for_standby=True, tries=3):
        """Prepares the drone for flight. returns True if process is successful returns False otherwise.

        Args:
            timeout: seconds wait at each step until timeout. default 30.
            wait_for_standby: whether to wait for standby or not. Defaults to 30.
            tries: How many times the function tries to set up.
        """

        # request all needed messages
        self.set_all_message_interval()

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
                self.ready_for_takeoff = True
                self.get_logger().info("Drone is ready for flight")
                return True

            self.get_logger().info(
                f"Drone Setup failed armed:{last_state.armed} guided:{last_state.guided} mode:{last_state.mode}"
            )

        return False

    def wait_for_new_state_message(self, timeout=30):
        """
        Wait for new state message to be received.  These are sent at
        1Hz so calling this is roughly equivalent to one-second delay.

        Args:
            timeout: Seconds to wait until timeout.

        """
        start_len = len(self.drone_state_messages)
        for _ in range(timeout):
            rclpy.spin_once(self)
            if len(self.drone_state_messages) > start_len:
                break

    def set_all_message_interval(self) -> None:
        """
        Requests data from the drone flight controller in the form of MavLink messages.
        Requested messages will be sent at periodic intervals, which are then processed by
        mavros and published to topics. This function makes async calls to "/mavros/set_message_interval service"
        to request these messages. Which then sends a MAV_CMD_SET_MESSAGE_INTERVAL commands to the drone.

        MavLink message ids can be found here: https://mavlink.io/en/messages/common.html

        Returns:
            None
        """
        for msg_id, msg_interval in self.messages_to_request:
            cmd = MessageInterval.Request()
            cmd.message_id = msg_id
            cmd.message_rate = msg_interval
            future = self.message_interval_cli.call_async(cmd)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info(
                    f"Set message interval result for msg with id:{msg_id} - {future.result()}"
                )
            else:
                self.get_logger().error(
                    f"Failed to call set_message_interval service for msg with id:{msg_id}"
                )

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

    def takeoff(self, target_alt: float = 3.0) -> bool:
        """Takeoff to the target altitude

        Args:
            target_alt (float): target altitude the drone will fly to when taking off. In meters.

        Returns:
            True if successful, False otherwise.

        """
        if self.in_air:
            self.get_logger().error("Could not takeoff! Drone is already in the air.")
            return False

        if not self.ready_for_takeoff:
            self.get_logger().error("Drone is not ready for takeoff.")
            return False

        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = float(target_alt)
        future = self.takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)

        # wait until takeoff is completed
        self.in_air = True
        self.in_air_start_time = self.get_clock().now()
        return True


def main(args=None):
    rclpy.init(args=args)
    drone_follower_node = DroneFollowerNode()

    try:
        drone_follower_node.setup_for_flight()
        drone_follower_node.takeoff()
        rclpy.spin(drone_follower_node)
    except KeyboardInterrupt:
        print(f"Keyboard interrupt detected. Shutting down")
    except Exception as e:
        print(f"Exception occurred while spinning the node - {e}")
    finally:
        drone_follower_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()