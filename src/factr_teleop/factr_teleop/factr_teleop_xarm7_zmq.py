#!/usr/bin/env python3
"""
factr_teleop_xarm7_zmq.py
--------------------------

IMPORTANT: run xarm7_zmq_server.py FIRST. This node will block waiting for
its first external-torque message otherwise (see set_up_communication).

Uses xarm7_left_real_zmq_addresses / xarm7_right_real_zmq_addresses from
python_utils/global_configs.py. Launch xarm7_zmq_server.py on the arm-side machine with
matching addresses:

    python3 xarm7_zmq_server.py --ip <xarm7 controller IP> --urdf .../xarm7.urdf \\
        --cmd_addr tcp://<sim_desktop_ip_address>:2098 \\
        --state_addr tcp://<sim_desktop_ip_address>:3099 \\
        --torque_addr tcp://<sim_desktop_ip_address>:3087


"""
import time
import numpy as np

import rclpy
from sensor_msgs.msg import JointState

from factr_teleop.factr_teleop_xArm7 import FACTRTeleop
from factr_teleop.python_utils.python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from python_utils.global_configs import xarm7_left_real_zmq_addresses, xarm7_right_real_zmq_addresses

NUM_ARM_JOINTS = 7


def create_array_msg(data):
    msg = JointState()
    msg.position = list(map(float, data))
    return msg


class FACTRTeleopXArm7ZMQ(FACTRTeleop):
    """
    Communication with the xArm7 follower over ZMQ, matching the pattern used
    for Franka. 
    """

    def __init__(self):
        super().__init__()
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]
        self.gripper_external_torque = 0.0

    def set_up_communication(self):
        if self.name == "left":
            zmq_addresses = xarm7_left_real_zmq_addresses
        elif self.name == "right":
            zmq_addresses = xarm7_right_real_zmq_addresses
        else:
            raise ValueError(f"Invalid robot name '{self.name}'. Expected 'left' or 'right'.")

        #ZMQ publisher: send leader arm joint position targets to the xarm7_zmq_server
        self.xarm_cmd_pub = ZMQPublisher(zmq_addresses["joint_pos_cmd_pub"])
        # ZMQ subscriber: current follower joint position from xarm7_zmq_server
        self.xarm_joint_state_sub = ZMQSubscriber(zmq_addresses["joint_state_sub"])

        #ROS publishers for logging / data collection, matching the Franka pattern
        self.obs_xarm7_state_pub = self.create_publisher(
            JointState,
            f'/xarm7/{self.name}/obs_xarm7_state',
            10
        )
        self.cmd_xarm7_pos_pub = self.create_publisher(
            JointState,
            f'/factr_teleop/{self.name}/cmd_xarm7_pos',
            10
        )
        self.cmd_gripper_pos_pub = self.create_publisher(
            JointState,
            f'/factr_teleop/{self.name}/cmd_gripper_pos',
            10
        )

        if self.enable_torque_feedback:
            #ZMQ subscriber pre-computed external joint torque
            self.xarm_torque_sub = ZMQSubscriber(zmq_addresses["joint_torque_sub"])
            self.get_logger().info(f"Waiting for xArm7 {self.name}'s external joint torque...")
            while self.xarm_torque_sub.message is None and rclpy.ok():
                self.get_logger().info(f"Still waiting for xarm7_zmq_server on {self.name}...")
                time.sleep(0.5)
            self.get_logger().info(f"Received xArm7 {self.name}'s external joint torque. Ready.")
            self.obs_xarm7_torque_pub = self.create_publisher(
                JointState,
                f'/xarm7/{self.name}/obs_xarm7_torque',
                10
            )

        if self.enable_gripper_feedback:
            self.obs_gripper_torque_pub = self.create_subscription(
                JointState, f'/gripper/{self.name}/obs_gripper_torque',
                self._gripper_external_torque_callback,
                1,
            )

    def _gripper_external_torque_callback(self, data):
        gripper_external_torque = data.position[0]
        self.gripper_external_torque = self.gripper_torque_ema_beta * self.gripper_external_torque + \
            (1 - self.gripper_torque_ema_beta) * gripper_external_torque

    def get_leader_gripper_feedback(self):
        return self.gripper_external_torque

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0 * gripper_feedback / self.gripper_feedback_gain
        return torque_gripper

    def get_leader_arm_external_joint_torque(self):
        external_torque = self.xarm_torque_sub.message
        if external_torque is None:
            external_torque = np.zeros(NUM_ARM_JOINTS)
        else:
            external_torque = np.asarray(external_torque[:NUM_ARM_JOINTS])
        self.obs_xarm7_torque_pub.publish(create_array_msg(external_torque))
        return external_torque

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        #send leader arm position as the joint position target for the xarm7_zmq_server
        self.xarm_cmd_pub.send_message(np.asarray(leader_arm_pos, dtype=np.float64))
        self.cmd_xarm7_pos_pub.publish(create_array_msg(leader_arm_pos))
        self.cmd_gripper_pos_pub.publish(create_array_msg([leader_gripper_pos]))

        #re-publish the follower's current joint state to ROS for logging
        xarm_state = self.xarm_joint_state_sub.message
        if xarm_state is not None:
            self.obs_xarm7_state_pub.publish(create_array_msg(xarm_state))


def main(args=None):
    rclpy.init(args=args)
    node = FACTRTeleopXArm7ZMQ()
    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
        node.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
