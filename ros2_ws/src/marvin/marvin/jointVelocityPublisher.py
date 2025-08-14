import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog

class JointErrorToVelocityPublisher(Node):
    """
    Subscribes to: [topic] /joint_goals, /joint_states
    Publishes to: [topic] /intermediate_joint_cmds (JointJog formatting for MoveIt Servo usage)
    """
    def __init__(self):
        super().__init__('joint_error_to_velocity_publisher')

        self.joint_goal = None
        self.joint_actual = None
        self.gain = 10.0  # Velocity input gain multiplier, need to be tuned
        self.velocity_limit = 4.8  # Joint velocity limit, need to be tuned

        # Subscriptions
        self.subscription_goal = self.create_subscription(
            JointState,
            'joint_goals',
            self.joint_goal_callback,
            10
        )

        self.subscription_actual = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_actual_callback,
            10
        )

        # Publisher to MoveIt Servo's expected input
        self.joint_velocity_publisher = self.create_publisher(
            JointJog,
            'intermediate_joint_cmds',
            10
        )

    def joint_goal_callback(self, msg):
        self.joint_goal = msg
        self.calculate_and_publish_velocity()

    def joint_actual_callback(self, msg):
        self.joint_actual = msg
        self.calculate_and_publish_velocity()

    def calculate_and_publish_velocity(self):
        if self.joint_goal is None or self.joint_actual is None:
            return

        goal_dict = dict(zip(self.joint_goal.name, self.joint_goal.position))
        actual_dict = dict(zip(self.joint_actual.name, self.joint_actual.position))

        joint_errors = {}
        joint_velocities = {} 

        for joint_name in goal_dict:
            if joint_name in actual_dict:
                error = goal_dict[joint_name] - actual_dict[joint_name]
                velocity = np.clip(self.gain * error, -self.velocity_limit, self.velocity_limit)
                joint_errors[joint_name] = error
                joint_velocities[joint_name] = velocity
                # want adduction (joint 2) specifically to have higher gain
                if joint_name in ('left_joint2', 'right_joint2'):
                    joint_velocities[joint_name] *= 2.0
            else:
                self.get_logger().warn(f"Joint '{joint_name}' not found in actual joint states.")

        # Log errors 
        self.get_logger().info(f"Joint Errors: {joint_errors}")
        self.publish_joint_velocity(joint_velocities)


    def publish_joint_velocity(self, joint_velocities):
        jog_msg = JointJog()
        jog_msg.header.stamp = self.get_clock().now().to_msg()
        jog_msg.joint_names = list(joint_velocities.keys())
        jog_msg.velocities = list(joint_velocities.values())
        jog_msg.duration = 0.1 #This is optional, but can be used to set the duration of the jog command

        self.joint_velocity_publisher.publish(jog_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointErrorToVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
