# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "drive_controller")
        self.declare_parameter("publishing_rate_in_hz", 25)
        self.declare_parameter("wait_sec_between_profiles", 1)
        self.declare_parameter("vel_names", ["vel1", "vel2"])
        self.declare_parameter("acc_names", ["acc1", "acc2"])
        self.declare_parameter("joints", ["joint1", "joint2"])

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        publishing_rate_in_hz = self.get_parameter("publishing_rate_in_hz").value
        wait_sec_between_profiles = self.get_parameter("wait_sec_between_profiles").value
        vel_names = self.get_parameter("vel_names").value
        acc_names = self.get_parameter("acc_names").value
        self.joints = self.get_parameter("joints").value

        self.segment_duration_in_seconds = 1.0

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        self.joint_state_msg_received = False

        # Read all velocities from parameters
        self.velocities = []
        for name in vel_names:
            self.get_logger().debug(
                'Extracting velocities for goal {}'.format(name)
            )

            self.declare_parameter(name)
            velocity = self.get_parameter(name).value
            if velocity is None or len(velocity) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_velocities = []
            for value in velocity:
                float_velocities.append(float(value))
            self.velocities.append(float_velocities)

        # Read all accelerations from parameters
        self.accelerations = []
        for name in acc_names:
            self.get_logger().debug(
                'Extracting accelerations for goal {}'.format(name)
            )

            self.declare_parameter(name)
            acceleration = self.get_parameter(name).value
            if acceleration is None or len(acceleration) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_accelerations = []
            for value in acceleration:
                float_accelerations.append(float(value))
            self.accelerations.append(float_accelerations)

        self.profile_duration = self.segment_duration_in_seconds * len(self.velocities)
        self.get_logger().info(
            'Profile duration set to: {} s'.format(self.profile_duration)
        )

        self.profile_and_wait_duration = self.profile_duration + wait_sec_between_profiles
        self.get_logger().info(
            'Profile and wait duration set to: {} s'.format(self.profile_and_wait_duration)
        )

        publish_topic = "/" + controller_name + "/" + "commands"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" at {} Hz'.format(
                len(vel_names), publish_topic, publishing_rate_in_hz
            )
        )

        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)

        self.sequence_start_time = self.get_clock().now()
        self.timer = self.create_timer(
            1.0 / publishing_rate_in_hz,
            self.timer_callback,
            callback_group=None,
            clock=self.get_clock())
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(
            'Timer callback called ..'
        )

        current_time = self.get_clock().now()
        trajectory_running_duration: Duration = current_time - self.sequence_start_time
        self.get_logger().info(
            'Current trajectory duration {} s. Based on current time {} and sequence start time {}'.format(
                trajectory_running_duration,
                current_time,
                self.sequence_start_time
            )
        )

        running_duration_as_float: float = trajectory_running_duration.nanoseconds * 1e-9
        self.get_logger().info(
            'Current trajectory duration {} s'.format(running_duration_as_float)
        )

        if running_duration_as_float > self.profile_and_wait_duration:
            self.sequence_start_time = self.get_clock().now()

            self.get_logger().info(
                'Trajectory finished resetting start time to: {}'.format(self.sequence_start_time)
            )
            return

        if running_duration_as_float > self.profile_duration:
            self.get_logger().info(
                'Trajectory completed waiting for restart time. Current duration {} s. Desired total time {}'.format(running_duration_as_float, self.profile_duration)
            )
            return

        self.get_logger().info(
            'Calculating next step in profile at time {} s'.format(running_duration_as_float)
        )

        lower_bound_of_profile_section = int(running_duration_as_float)
        upper_bound_of_profile_section = lower_bound_of_profile_section + 1

        time_fraction = (running_duration_as_float - lower_bound_of_profile_section) / self.segment_duration_in_seconds

        if (lower_bound_of_profile_section < 0):
            self.get_logger().info(
                'Starting profile index out of range. Index is {}. Ignoring'.format(lower_bound_of_profile_section)
            )
            return

        if (upper_bound_of_profile_section >= len(self.velocities)):
            self.get_logger().info(
                'Ending profile index out of range. Index is {}. Ignoring'.format(upper_bound_of_profile_section)
            )
            return

        profile_start_values = self.velocities[lower_bound_of_profile_section]
        profile_end_values = self.velocities[upper_bound_of_profile_section]

        # Find the section we're publishing, assuming that each section takes 1 second
        # 0 - 1 -> first

        values: List[float] = []
        for i in range(len(profile_start_values)):
            start = profile_start_values[i]
            end = profile_end_values[i]

            value = (end - start) * time_fraction + start
            values.append(value)

        msg = Float64MultiArray()
        msg.data = values

        self.get_logger().info(
            'Publishing movement command {} '.format(msg)
        )

        self.publisher_.publish(msg)

        self.i += 1
        self.i %= len(self.velocities)

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:
            self.joint_state_msg_received = True
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    pub = DriveController()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
