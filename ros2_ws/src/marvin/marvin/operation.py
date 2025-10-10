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
from custom_interfaces.msg import HandLandmark


# Gripper positions
OPEN_POSITION  = 0.019  # Open gripper position
CLOSED_POSITION = -0.01  # Closed gripper position

LEFT_ARM_JOINT_TOPIC        = '/left/servo_node/delta_joint_cmds'
RIGHT_ARM_JOINT_TOPIC       = '/right/servo_node/delta_joint_cmds'
INTERMEDIATE_JOINT_TOPIC    = '/intermediate_joint_cmds'
INTERMEDIATE_HAND_TOPIC     = '/hand_landmarks'
RIGHT_GRIPPER_ACTION        = '/right_hand_controller/gripper_cmd'
LEFT_GRIPPER_ACTION         = '/left_hand_controller/gripper_cmd'
LEFT_START_SERVO_SRV        = '/left/servo_node/start_servo'
LEFT_STOP_SERVO_SRV         = '/left/servo_node/stop_servo'
RIGHT_START_SERVO_SRV       = '/right/servo_node/start_servo'
RIGHT_STOP_SERVO_SRV        = '/right/servo_node/stop_servo'
BASE_FRAME_ID               = 'torso'
PUBLISH_RATE_HZ             = 100


class TeleopNode(Node):

    def __init__(self):
        super().__init__('open_manipulator_x_teleop')

        # Publisher for MoveIt Servo
        self.left_joint_pub = self.create_publisher(JointJog, LEFT_ARM_JOINT_TOPIC, 10)
        self.right_joint_pub = self.create_publisher(JointJog, RIGHT_ARM_JOINT_TOPIC, 10)

        # Subscriber for external JointJog
        self.create_subscription(
            JointJog, INTERMEDIATE_JOINT_TOPIC,
            self.external_joint_cb, 10
        )

        # Subscriber for hand landmarks (simple open/close gripper control)
        self.create_subscription(
            HandLandmark, INTERMEDIATE_HAND_TOPIC,
            self.hand_cb, 10
        )

        # Left gripper action client
        self.left_gripper_ac = ActionClient(self, GripperCommand, LEFT_GRIPPER_ACTION)

        # Right gripper action client
        self.right_gripper_ac = ActionClient(self, GripperCommand, RIGHT_GRIPPER_ACTION)

        # Service clients for start/stop servo
        self.left_start_cli = self.create_client(Trigger, LEFT_START_SERVO_SRV)
        self.left_stop_cli  = self.create_client(Trigger, LEFT_STOP_SERVO_SRV)
        self.right_start_cli = self.create_client(Trigger, RIGHT_START_SERVO_SRV)
        self.right_stop_cli  = self.create_client(Trigger, RIGHT_STOP_SERVO_SRV)

        # Spin in background to service callbacks
        threading.Thread(target=self._spin_loop, daemon=True).start()

        # Wait/connect to MoveIt Servo services
        self._wait_srv(self.left_start_cli, 'left_start_servo')
        self._wait_srv(self.left_stop_cli,  'left_stop_servo')
        self._call_trigger(self.left_start_cli, 'left_start')
        self._wait_srv(self.right_start_cli, 'right_start_servo')
        self._wait_srv(self.right_stop_cli,  'right_stop_servo')
        self._call_trigger(self.right_start_cli, 'right_start')


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
        names = msg.joint_names
        velocities = msg.velocities
        out_left = JointJog()
        out_right = JointJog()
        out_left.header.stamp = self.get_clock().now().to_msg()
        out_right.header.stamp = self.get_clock().now().to_msg()
        out_left.header.frame_id = BASE_FRAME_ID #pretty sure this is actually left/right_link1 so idk if need to add side to name
        out_right.header.frame_id = BASE_FRAME_ID
        for name, vel in zip(names, velocities):
            if name.startswith('left_'):
                out_left.velocities.append(vel)
                out_left.joint_names.append(name)
            elif name.startswith('right_'):
                out_right.velocities.append(vel)
                out_right.joint_names.append(name)
            else:
                self.get_logger().warn(f'Ignoring joint "{name}" not starting with "left_" or "right_"')
        self.left_joint_pub.publish(out_left)
        self.right_joint_pub.publish(out_right)
        self.get_logger().info(f'Forwarded external JointJog: {msg.joint_names}')


    def send_gripper_goal(self, side: str, position: float):
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = 100.0
        self.get_logger().info(f'Sending {side} gripper goal: {position:.3f}')
        if side == 'left':
            self.left_gripper_ac.wait_for_server()
            self.left_gripper_ac.send_goal_async(goal)
        elif side == 'right':
            self.right_gripper_ac.wait_for_server()
            self.right_gripper_ac.send_goal_async(goal)


    def hand_cb(self, msg: HandLandmark):
        """
        msg.label  = ['left_hand', 'right_hand']
        msg.status = [ True/False,  True/False ]
        """
        for hand_label, is_open in zip(msg.label, msg.status):
            if hand_label == 'left_hand':
                target = OPEN_POSITION  if is_open else CLOSED_POSITION
                self.send_gripper_goal('left', target)
                self.get_logger().info(f"Left hand is {'open' if is_open else 'closed'}, sending gripper command: {target:.3f}")
            elif hand_label == 'right_hand':
                target = OPEN_POSITION  if is_open else CLOSED_POSITION
                self.send_gripper_goal('right', target)
                self.get_logger().info(f"Right hand is {'open' if is_open else 'closed'}, sending gripper command: {target:.3f}")
    

    def destroy_node(self):
        self._call_trigger(self.stop_cli, 'stop')
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    signal.signal(signal.SIGINT, lambda s,f: rclpy.shutdown())
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
