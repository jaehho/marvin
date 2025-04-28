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
from control_msgs.msg import JointJog, GripperCommand as GripperCmdMsg
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration

# Key definitions
KEY_1 = '1'
KEY_2 = '2'
KEY_3 = '3'
KEY_4 = '4'
KEY_Q = 'q'
KEY_W = 'w'
KEY_E = 'e'
KEY_R = 'r'
KEY_O = 'o'
KEY_P = 'p'
KEY_ESC = '\x1b'

ARM_JOINT_TOPIC = '/servo_node/delta_joint_cmds'
GRIPPER_ACTION = 'gripper_controller/gripper_cmd'
START_SERVO_SRV = '/servo_node/start_servo'
STOP_SERVO_SRV = '/servo_node/stop_servo'
BASE_FRAME_ID = 'link1'
ARM_JOINT_VEL = 3.0  # rad/s
PUBLISH_RATE_HZ = 100

class KeyboardReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def read_key(self):
        return sys.stdin.read(1)

    def shutdown(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.old_settings)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('open_manipulator_x_teleop')
        # Service clients
        self.start_cli = self.create_client(Trigger, START_SERVO_SRV)
        self.stop_cli  = self.create_client(Trigger, STOP_SERVO_SRV)
        # Publisher
        self.joint_pub = self.create_publisher(JointJog, ARM_JOINT_TOPIC, 10)
        # Action client
        self.gripper_ac = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        # State
        self.joint_msg = JointJog()
        self.publish_joint = False

        # Start background threads
        threading.Thread(target=self._spin_loop, daemon=True).start()
        threading.Thread(target=self._pub_loop, daemon=True).start()

        # Wait for MoveIt Servo services
        self._wait_for_service(self.start_cli, 'start_servo')
        self._wait_for_service(self.stop_cli,  'stop_servo')
        self._call_trigger(self.start_cli, 'start')

    def _wait_for_service(self, client, name):
        self.get_logger().info(f'Waiting for moveit_servo {name} service...')
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Failed to connect to {name} service')
        else:
            self.get_logger().info(f'Connected to {name} service')

    def _call_trigger(self, client, name):
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result().success:
            self.get_logger().info(f'Successfully called {name} service')
        else:
            self.get_logger().warn(f'Could not call {name} service')

    def _spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _pub_loop(self):
        rate = self.create_rate(PUBLISH_RATE_HZ)
        while rclpy.ok():
            if self.publish_joint:
                self.joint_msg.header.stamp = self.get_clock().now().to_msg()
                self.joint_msg.header.frame_id = BASE_FRAME_ID
                self.joint_pub.publish(self.joint_msg)
                self.publish_joint = False
                self.get_logger().info('Published JointJog')
            rate.sleep()

    def send_gripper_goal(self, position: float):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 100.0
        self.get_logger().info(f'Sending gripper goal: {position:.3f}')
        self.gripper_ac.wait_for_server()
        self.gripper_ac.send_goal_async(goal_msg)

    def run(self):
        kb = KeyboardReader()
        self.get_logger().info('--- Open Manipulator X Teleop ---')
        self.get_logger().info('1/q: Joint1 +/-   2/w: Joint2 +/-   3/e: Joint3 +/-   4/r: Joint4 +/-')
        self.get_logger().info('o: open gripper   p: close gripper   ESC: quit')

        try:
            while rclpy.ok():
                c = kb.read_key()
                # reset
                self.joint_msg = JointJog()
                self.joint_msg.joint_names = []
                self.joint_msg.velocities = []

                if c == KEY_1:
                    self.joint_msg.joint_names.append('joint1')
                    self.joint_msg.velocities.append( ARM_JOINT_VEL)
                    self.publish_joint = True
                elif c == KEY_Q:
                    self.joint_msg.joint_names.append('joint1')
                    self.joint_msg.velocities.append(-ARM_JOINT_VEL)
                    self.publish_joint = True

                elif c == KEY_2:
                    self.joint_msg.joint_names.append('joint2')
                    self.joint_msg.velocities.append( ARM_JOINT_VEL)
                    self.publish_joint = True
                elif c == KEY_W:
                    self.joint_msg.joint_names.append('joint2')
                    self.joint_msg.velocities.append(-ARM_JOINT_VEL)
                    self.publish_joint = True

                elif c == KEY_3:
                    self.joint_msg.joint_names.append('joint3')
                    self.joint_msg.velocities.append( ARM_JOINT_VEL)
                    self.publish_joint = True
                elif c == KEY_E:
                    self.joint_msg.joint_names.append('joint3')
                    self.joint_msg.velocities.append(-ARM_JOINT_VEL)
                    self.publish_joint = True

                elif c == KEY_4:
                    self.joint_msg.joint_names.append('joint4')
                    self.joint_msg.velocities.append( ARM_JOINT_VEL)
                    self.publish_joint = True
                elif c == KEY_R:
                    self.joint_msg.joint_names.append('joint4')
                    self.joint_msg.velocities.append(-ARM_JOINT_VEL)
                    self.publish_joint = True

                elif c == KEY_O:
                    self.send_gripper_goal(0.019)
                elif c == KEY_P:
                    self.send_gripper_goal(-0.01)

                elif c == KEY_ESC:
                    self.get_logger().info('ESC pressed: shutting down')
                    break

                else:
                    self.get_logger().warn(f'Unknown key: 0x{ord(c):02X}')

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            kb.shutdown()
            # stop servo
            self._call_trigger(self.stop_cli, 'stop')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopNode()
    # Handle Ctrl-C at Python level
    signal.signal(signal.SIGINT, lambda sig, frame: rclpy.shutdown())
    teleop.run()


if __name__ == '__main__':
    main()
