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

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherVelocity(Node):
    def __init__(self):
        super().__init__("publisher_velocity_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "velocity_controller")
        self.declare_parameter("wait_sec_between_publish", 6)
        self.declare_parameter("goal_names", ["vel1", "vel2"])
        self.declare_parameter("joints", ["joint1", "joint2"])

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # initialize starting point status
        if not self.check_starting_point:
            self.starting_point_ok = True
        else:
            self.starting_point_ok = False

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            self.get_logger().debug(
                'Extracting positions for goal {}'.format(name)
            )

            self.declare_parameter(name)
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names), publish_topic, wait_sec_between_publish
            )
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(
            wait_sec_between_publish,
            self.timer_callback,
            callback_group=None,
            clock=self.get_clock())
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(
            'Timer callback called ..'
        )

        traj = JointTrajectory()
        traj.joint_names = self.joints
        #traj.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(self.goals)):
            point = JointTrajectoryPoint()
            point.velocities = self.goals[i]
            time = i * 1 + 1
            point.time_from_start = Duration(sec=time)

            traj.points.append(point)

        self.get_logger().info(
            'Publishing movement command {} '.format(traj)
        )

        self.publisher_.publish(traj)

        self.i += 1
        self.i %= len(self.goals)

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:
            self.joint_state_msg_received = True
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    pub = PublisherVelocity()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
