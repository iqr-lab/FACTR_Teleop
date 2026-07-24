#!/usr/bin/env python3
"""
xarm7_zmq_server.py
--------------------
Follower-side server for a real xArm7, mirroring the role that the Franka
control PC / libfranka realtime loop plays in factr_teleop_franka_zmq.py.


Usage (right arm, matching this workspace's existing global_configs.py):
  python3 xarm7_zmq_server.py \\
      --ip <xarm7 controller IP -- SDK connection, e.g. from launch args> \\
      --urdf /path/to/xarm7.urdf \\
      --cmd_addr tcp://<sim_desktop_ip_address>:2098 \\
      --state_addr tcp://192.168.1.205:3099 \\
      --torque_addr tcp://192.168.1.205:3087

These three addresses must match xarm7_left_real_zmq_addresses /
xarm7_right_real_zmq_addresses in python_utils/global_configs.py, which
already exists in this workspace. NOTE: --ip (the SDK's connection target)
is a separate concept from the *_ip_address values in that config file --
the latter are the ZMQ server host's own network address (what it binds
to), which is not guaranteed to be the same machine/IP as the xArm7
controller itself. Confirm this for your physical setup rather than
assuming --ip == xarm7_right_ip_address.

NOTE ON MESH-FREE MODEL LOADING: this uses pin.buildModelFromUrdf (singular),
not buildModelsFromUrdf, so mesh file paths never need to resolve -- only the
URDF's <inertial> tags matter for RNEA.
"""
import argparse
import signal
import sys
import time

import numpy as np
import pinocchio as pin
from xarm.wrapper import XArmAPI

from factr_teleop.python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber

NUM_ARM_JOINTS = 7
DEFAULT_MAX_DELTA = 0.05  # rad per command step, same as gello's XArmRobot


class XArm7ZMQServer:
    def __init__(self, ip, urdf_path, cmd_addr, state_addr, torque_addr,
                 rate_hz=200.0, torque_ema_beta=0.2, max_delta=DEFAULT_MAX_DELTA):
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.max_delta = max_delta
        self.torque_ema_beta = torque_ema_beta
        self.prev_tau_ext = np.zeros(NUM_ARM_JOINTS)

        #dynamics model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        print(f"[xarm7_zmq_server] pinocchio joint order: {list(self.model.names)}")

        #ZMQ: server binds the state/torque topics and connects to the cmd topic 
        self.state_pub = ZMQPublisher(state_addr)
        self.torque_pub = ZMQPublisher(torque_addr)
        self.cmd_sub = ZMQSubscriber(cmd_addr)

        #xArm SDK setup
        self.arm = XArmAPI(ip, is_radian=True)
        self._init_arm()

        self.running = True
        self._last_target = None

    def _init_arm(self):
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(True)
        time.sleep(0.5)
        self.arm.set_mode(1)  # servoj streaming mode
        time.sleep(0.5)
        self.arm.set_state(0)
        time.sleep(0.5)
        # 0 = report estimated joint torque (N.m); 1 = raw motor current (A).
        # Force this explicitly rather than trusting the controller's default.
        ret = self.arm.set_report_tau_or_i(0)
        print(f"[xarm7_zmq_server] set_report_tau_or_i(0) -> ret={ret}")

    def _read_state(self):
        """Returns (q, qdot, tau_measured) or None if the SDK call failed."""
        code, states = self.arm.get_joint_states(is_radian=True)
        if code != 0 or states is None:
            print(f"[xarm7_zmq_server] get_joint_states error code {code}")
            return None
        q = np.array(states[0][:NUM_ARM_JOINTS])
        qdot = np.array(states[1][:NUM_ARM_JOINTS])
        tau_measured = np.array(states[2][:NUM_ARM_JOINTS])
        return q, qdot, tau_measured

    def _command_step(self, q_current):
        target = self.cmd_sub.message
        if target is None:
            return
        target = np.asarray(target[:NUM_ARM_JOINTS], dtype=float)
        delta = target - q_current
        norm = np.linalg.norm(delta)
        if norm > self.max_delta:
            delta = delta / norm * self.max_delta
        cmd = q_current + delta
        ret = self.arm.set_servo_angle_j(cmd, is_radian=True)
        if ret in (1, 9):
            print(f"[xarm7_zmq_server] set_servo_angle_j error {ret}, clearing")
            self._init_arm()

    def run(self):
        print(f"[xarm7_zmq_server] running at {self.rate_hz} Hz. Ctrl+C to stop.")
        while self.running:
            t0 = time.time()

            result = self._read_state()
            if result is not None:
                q, qdot, tau_measured = result

                tau_model = pin.rnea(self.model, self.data, q, qdot, np.zeros(NUM_ARM_JOINTS))
                tau_ext_raw = tau_measured - tau_model
                tau_ext = (
                    self.torque_ema_beta * self.prev_tau_ext
                    + (1 - self.torque_ema_beta) * tau_ext_raw
                )
                self.prev_tau_ext = tau_ext

                self.state_pub.send_message(q)
                self.torque_pub.send_message(tau_ext)

                self._command_step(q)

            elapsed = time.time() - t0
            time.sleep(max(0.0, self.period - elapsed))

    def stop(self):
        print("[xarm7_zmq_server] stopping, disabling arm...")
        self.running = False
        try:
            self.arm.set_mode(0)
            self.arm.set_state(4)  # stop
            self.arm.motion_enable(False)
        except Exception as e:
            print(f"[xarm7_zmq_server] error during shutdown: {e}")
        self.arm.disconnect()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--cmd_addr', required=True, help='ZMQ address the leader PUBLISHES commands on')
    parser.add_argument('--state_addr', required=True, help='ZMQ address THIS server publishes joint state on')
    parser.add_argument('--torque_addr', required=True, help='ZMQ address THIS server publishes external torque on')
    parser.add_argument('--rate', type=float, default=200.0)
    parser.add_argument('--torque_ema_beta', type=float, default=0.2)
    parser.add_argument('--max_delta', type=float, default=DEFAULT_MAX_DELTA)
    args = parser.parse_args()

    server = XArm7ZMQServer(
        ip=args.ip,
        urdf_path=args.urdf,
        cmd_addr=args.cmd_addr,
        state_addr=args.state_addr,
        torque_addr=args.torque_addr,
        rate_hz=args.rate,
        torque_ema_beta=args.torque_ema_beta,
        max_delta=args.max_delta,
    )

    def _handle_sigint(sig, frame):
        server.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _handle_sigint)
    server.run()


if __name__ == '__main__':
    main()