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
        self.sides = ['left','right']
        self.joint_types = ['shoulder_flexion', 'shoulder_adduction', 'elbow_flexion']
        #self.joint_names = [f'{side}_{joint_type}' for side in self.sides for joint_type in self.joint_types]
        
        # Initialize a dictionary to keep track of the latest joint angles with default 0.0
        #self.joint_angles = {joint_name: 0.0 for joint_name in self.joint_names}

        # Latest commanded angles (rad)
        self.joint_angles = {
            f'{side}_{jt}': 0.0
            for side in self.sides
            for jt in self.joint_types
        }
        
        # Create subscriptions dynamically
        #for joint_name in self.joint_names:
            #self.create_subscription(Float64, joint_name, self.create_handler(joint_name), 10)

        for side in self.sides:
            for jt in self.joint_types:
                topic = f'{side}_{jt}'  # no leading slash ⇒ relative to namespace
                self.create_subscription(Float64, topic, self._make_handler(side, jt), 10)
        
        # Fixed output joint order: left then right
        self.robot_joint_names = [
            'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4',
            'left_gripper', 'left_gripper_sub',
            'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4',
            'right_gripper', 'right_gripper_sub',
        ]
        
        self.joint_goal_publisher = self.create_publisher(JointState, 'joint_goals', 10)

        # Joint limits used for clipping (rad)
        self.limits = {
            'joint1': (-2.82743, 2.82743),
            'joint2': (-1.79071, 1.57080),
            'joint3': (-0.94248, 1.38230),
        }


    # def create_handler(self, joint_name):
    #     def handler(msg):
    #         self.joint_angles[joint_name] = msg.data
    #         self.publish_joint_goal()
    #     return handler

    def _make_handler(self, side, joint_type):
        key = f'{side}_{joint_type}'
        def handler(msg: Float64):
            self.joint_angles[key] = float(msg.data)
            self.publish_joint_goal()
        return handler
    
    def _arm_positions(self, side: str):
        """Map human-friendly angles to robot joints 1–3 for one side."""
        # Shoulder flexion sign: left is negated (as in your code), right is positive
        flex = self.joint_angles[f'{side}_shoulder_flexion']
        shoulder_flexion = -flex if side == 'left' else flex
        joint1 = np.clip(shoulder_flexion, *self.limits['joint1'])

        # Shoulder adduction → robot joint2 (π/2 - adduction)
        add = self.joint_angles[f'{side}_shoulder_adduction']
        joint2 = np.clip(np.pi/2 - add, *self.limits['joint2'])

        # Elbow flexion → robot joint3 (π/2 - flexion)
        elbow = self.joint_angles[f'{side}_elbow_flexion']
        joint3 = np.clip(np.pi/2 - elbow, *self.limits['joint3'])

        # joint4 + grippers: placeholders (adjust if you wire these later)
        joint4 = 0.0
        gripper = 0.0
        gripper_sub = 0.0

        return [joint1, joint2, joint3, joint4, gripper, gripper_sub]

    # def publish_joint_goal(self):
    #     joint_goal_msg = JointState()
    #     joint_goal_msg.header = Header()
    #     joint_goal_msg.header.stamp = self.get_clock().now().to_msg()
        
    #     robot_joint_names = []
    #     joint_positions = []
        
    #     for side in self.sides:
    #         # robot_joint_names.extend([
    #         #     f'{side}_joint1', f'{side}_joint2', f'{side}_joint3',
    #         #     f'{side}_joint4', f'{side}_gripper'
    #         # ])
    #         robot_joint_names.extend([
    #             'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4', 'left_gripper', 'left_gripper_sub',
    #             'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4', 'right_gripper', 'right_gripper_sub',   
    #         ])
            
    #         if side == 'left':
    #             shoulder_flexion_angle = -self.joint_angles[f'{side}_shoulder_flexion']
    #         else:
    #             shoulder_flexion_angle = self.joint_angles[f'{side}_shoulder_flexion']
            
    #         joint_positions.extend([
    #             np.clip(shoulder_flexion_angle, -2.82743, 2.82743),
    #             np.clip(np.pi / 2 - self.joint_angles[f'{side}_shoulder_adduction'], -1.79071, 1.57080),
    #             np.clip(np.pi / 2 - self.joint_angles[f'{side}_elbow_flexion'], -0.94248, 1.38230),
    #             0.0, 0.0, 0.0,  # Placeholder for other joint values not computed
    #         ])
        
    #     joint_goal_msg.name = robot_joint_names
    #     joint_goal_msg.position = joint_positions
    #     joint_goal_msg.velocity = []
    #     joint_goal_msg.effort = []

    #     self.joint_goal_publisher.publish(joint_goal_msg)

    def publish_joint_goal(self):
        # Build positions in the same order as names: left..., right...
        left_positions = self._arm_positions('left')
        right_positions = self._arm_positions('right')

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.robot_joint_names
        msg.position = left_positions + right_positions
        msg.velocity = []
        msg.effort = []
        self.joint_goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_goal_publisher = JointGoalPublisher()
    rclpy.spin(joint_goal_publisher)
    joint_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
