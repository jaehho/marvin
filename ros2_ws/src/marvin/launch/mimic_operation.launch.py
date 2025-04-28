from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'marvin'

    # Common node parameters
    node_params = {
        'shoulderFlexion': [('left_shoulder_flexion', 'left'), ('right_shoulder_flexion', 'right')],
        'shoulderAdduction': [('left_shoulder_adduction', 'left'), ('right_shoulder_adduction', 'right')],
        'elbowFlexion': [('left_elbow_flexion', 'left'), ('right_elbow_flexion', 'right')]
    }

    ld = LaunchDescription([
        # Pose detection node
        Node(package=package_name, executable='poseDetection', output='screen'),

        # Pose Display node
        Node(package=package_name, executable='poseDisplay', output='screen'),

        # Operation node
        Node(package=package_name, executable='operation', output='screen'),
        
        # Joint velocity publisher node
        Node(package=package_name, executable='jointVelocityPublisher', output='screen'),

        # Joint state publisher node
        Node(package=package_name, executable='jointGoalPublisher', output='screen'),
    ])

    # Add shoulder flexion, adduction, and elbow flexion nodes
    for executable, names in node_params.items():
        for name, side in names:
            ld.add_action(Node(
                package=package_name,
                executable=executable,
                name=name,
                parameters=[{'side': side}],
                output='screen'
            ))

    return ld
