#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand

# Key definitions
KEY_O   = 'o'
KEY_P   = 'p'
KEY_ESC = '\x1b'

ARM_JOINT_TOPIC             = '/servo_node/delta_joint_cmds'
INTERMEDIATE_JOINT_TOPIC    = '/intermediate_joint_cmds'
GRIPPER_ACTION              = 'gripper_controller/gripper_cmd'
START_SERVO_SRV             = '/servo_node/start_servo'
STOP_SERVO_SRV              = '/servo_node/stop_servo'
BASE_FRAME_ID               = 'link1'
PUBLISH_RATE_HZ             = 100

class KeyboardReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
    def read_key(self):
        return sys.stdin.read(1)
    def shutdown(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.old)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('open_manipulator_x_teleop')

        # Publisher for MoveIt Servo
        self.joint_pub = self.create_publisher(JointJog, ARM_JOINT_TOPIC, 10)

        # Subscriber for external JointJog
        self.create_subscription(
            JointJog, INTERMEDIATE_JOINT_TOPIC,
            self.external_joint_cb, 10
        )

        # Gripper action client
        self.gripper_ac = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        # Service clients for start/stop servo
        self.start_cli = self.create_client(Trigger, START_SERVO_SRV)
        self.stop_cli  = self.create_client(Trigger, STOP_SERVO_SRV)

        # Spin in background to service callbacks
        threading.Thread(target=self._spin_loop, daemon=True).start()

        # Wait/connect to MoveIt Servo services
        self._wait_srv(self.start_cli, 'start_servo')
        self._wait_srv(self.stop_cli,  'stop_servo')
        self._call_trigger(self.start_cli, 'start')

    def _spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_srv(self, cli, name):
        self.get_logger().info(f'Waiting for {name} service...')
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Failed to connect to {name}')
        else:
            self.get_logger().info(f'Connected to {name}')

    def _call_trigger(self, cli, name):
        req = Trigger.Request()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.done() and fut.result().success:
            self.get_logger().info(f'{name} service succeeded')
        else:
            self.get_logger().warn(f'{name} service failed or timed out')

    def external_joint_cb(self, msg: JointJog):
        # Stamp and forward to servo
        out = JointJog()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = BASE_FRAME_ID
        out.joint_names = msg.joint_names
        out.velocities  = msg.velocities
        self.joint_pub.publish(out)
        self.get_logger().info(f'Forwarded external JointJog: {msg.joint_names}')

    def send_gripper_goal(self, position: float):
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = 100.0
        self.get_logger().info(f'Sending gripper goal: {position:.3f}')
        self.gripper_ac.wait_for_server()
        self.gripper_ac.send_goal_async(goal)

    def run(self):
        kb = KeyboardReader()
        self.get_logger().info('--- Gripper Teleop (joint cmds from topic) ---')
        self.get_logger().info('o: open gripper   p: close gripper   ESC: quit')
        try:
            while rclpy.ok():
                c = kb.read_key()
                if c == KEY_O:
                    self.send_gripper_goal(0.019)
                elif c == KEY_P:
                    self.send_gripper_goal(-0.01)
                elif c == KEY_ESC:
                    self.get_logger().info('ESC pressed: shutting down')
                    break
                else:
                    # ignore other keys
                    continue
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            kb.shutdown()
            self._call_trigger(self.stop_cli, 'stop')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    signal.signal(signal.SIGINT, lambda s,f: rclpy.shutdown())
    node.run()

if __name__ == '__main__':
    main()
