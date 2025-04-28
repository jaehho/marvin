import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointGoalPublisher(Node):
    """
    Subscribes to: [topic] /left_shoulder_flexion, /left_shoulder_adduction, /left_elbow_flexion,
                    /right_shoulder_flexion, /right_shoulder_adduction, /right_elbow_flexion
    Publishes to: [topic] /joint_goals
    """
    def __init__(self):
        super().__init__('joint_goal_publisher')
        
        # Define joint names for each side
        self.sides = ['left'] # TODO: Add 'right' side implementation
        self.joint_types = ['shoulder_flexion', 'shoulder_adduction', 'elbow_flexion']
        self.joint_names = [f'{side}_{joint_type}' for side in self.sides for joint_type in self.joint_types]
        
        # Initialize a dictionary to keep track of the latest joint angles with default 0.0
        self.joint_angles = {joint_name: 0.0 for joint_name in self.joint_names}
        
        # Create subscriptions dynamically
        for joint_name in self.joint_names:
            self.create_subscription(Float64, joint_name, self.create_handler(joint_name), 10)
        
        self.joint_goal_publisher = self.create_publisher(JointState, 'joint_goals', 10)

    def create_handler(self, joint_name):
        def handler(msg):
            self.joint_angles[joint_name] = msg.data
            self.publish_joint_goal()
        return handler

    def publish_joint_goal(self):
        joint_goal_msg = JointState()
        joint_goal_msg.header = Header()
        joint_goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        robot_joint_names = []
        joint_positions = []
        
        for side in self.sides:
            # robot_joint_names.extend([
            #     f'{side}_joint1', f'{side}_joint2', f'{side}_joint3',
            #     f'{side}_joint4', f'{side}_gripper'
            # ])
            robot_joint_names.extend([
                'joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint', 'gripper_right_joint'
            ])
            
            if side == 'left':
                shoulder_flexion_angle = -self.joint_angles[f'{side}_shoulder_flexion']
            else:
                shoulder_flexion_angle = self.joint_angles[f'{side}_shoulder_flexion']
            
            joint_positions.extend([
                np.clip(shoulder_flexion_angle, -2.82743, 2.82743),
                np.clip(np.pi / 2 - self.joint_angles[f'{side}_shoulder_adduction'], -1.79071, 1.57080),
                np.clip(np.pi / 2 - self.joint_angles[f'{side}_elbow_flexion'], -0.94248, 1.38230),
                0.0, 0.0, 0.0  # Placeholder for other joint values not computed
            ])
        
        joint_goal_msg.name = robot_joint_names
        joint_goal_msg.position = joint_positions
        joint_goal_msg.velocity = []
        joint_goal_msg.effort = []

        self.joint_goal_publisher.publish(joint_goal_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_goal_publisher = JointGoalPublisher()
    rclpy.spin(joint_goal_publisher)
    joint_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
