import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from custom_interfaces.msg import PoseLandmark
import numpy as np
from std_msgs.msg import Header  # Import the Header message
from sensor_msgs.msg import JointState  # Import the JointState message to publish the joint states
from rclpy.qos import qos_profile_sensor_data  # Import the sensor data QoS profile

def vector_from_points(p1, p2):
    """Create a vector from two points."""
    return np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])

def calculate_angle(v1, v2):
    """Calculate the angle between two vectors."""
    dot_product = np.dot(v1, v2)
    magnitude_product = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos_angle = dot_product / magnitude_product
    angle_radians = np.arccos(cos_angle)
    angle_degrees = np.degrees(angle_radians)
    return angle_degrees

class ShoulderAngleSubscriber(Node):

    def __init__(self):
        super().__init__('shoulder_angle_subscriber')
        self.subscription = self.create_subscription(
            PoseLandmark,
            'pose_landmarks',
            self.listener_callback,
            10)
        # Add a publisher for JointState messages
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

    def listener_callback(self, msg):
        labels = msg.label
        points = msg.point

        # Find indices for shoulders, elbows, and hips
        left_shoulder_idx = labels.index('left_shoulder')
        right_shoulder_idx = labels.index('right_shoulder')
        left_elbow_idx = labels.index('left_elbow')
        right_elbow_idx = labels.index('right_elbow')
        left_hip_idx = labels.index('left_hip')
        right_hip_idx = labels.index('right_hip')

        # Calculate vectors
        left_upper_arm = vector_from_points(points[left_shoulder_idx], points[left_elbow_idx])
        right_upper_arm = vector_from_points(points[right_shoulder_idx], points[right_elbow_idx])
        left_shoulder_to_hip = vector_from_points(points[left_shoulder_idx], points[left_hip_idx])
        right_shoulder_to_hip = vector_from_points(points[right_shoulder_idx], points[right_hip_idx])

        # Calculate angles using the shoulder-to-hip vector as reference
        left_shoulder_angle = calculate_angle(left_upper_arm, left_shoulder_to_hip)
        right_shoulder_angle = calculate_angle(right_upper_arm, right_shoulder_to_hip)

        left_shoulder_angle_radians = left_shoulder_angle * np.pi/180.0
        right_shoulder_angle_radians = right_shoulder_angle * np.pi/180.0

        # Prepare a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1','joint2','joint3','joint4','gripper','gripper_sub']  # Example names
        joint_state_msg.position = [0.0,left_shoulder_angle_radians,0.0,0.0,0.0,0.0]  # Assuming you calculate these
        joint_state_msg.velocity = []  # Leave empty if not used
        joint_state_msg.effort = []  # Leave empty if not used

        # Publish the JointState message
        self.joint_state_publisher.publish(joint_state_msg)

def main(args=None):
    print('running')
    rclpy.init(args=args)
    shoulder_angle_subscriber = ShoulderAngleSubscriber()
    rclpy.spin(shoulder_angle_subscriber)
    shoulder_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
