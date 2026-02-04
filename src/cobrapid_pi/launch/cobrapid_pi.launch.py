import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Déclaration de l'argument device
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Camera device ID'
    )

    # Définition du node pour camera_pose_broadcaster
    camera_tag_pose_broadcaster = Node(
        package='cobrapid_pi',
        executable='camera_tag_pose_broadcaster',
        name='tag_to_cam_tf',
        output='screen'
    )

    # Utiliser FindPackageShare pour obtenir le chemin du fichier de configuration
    apriltag_ros_share = FindPackageShare('apriltag_ros').find('apriltag_ros')
    apriltag_config_path = os.path.join(apriltag_ros_share, 'cfg', 'tags_36h11.yaml')

    # Définition du container pour les nodes composables (caméra, rectification et apriltag)
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Node de la caméra
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera',
                namespace='camera',
                parameters=[{'camera': LaunchConfiguration('device')}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Node de rectification d'image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace='camera',
                remappings=[
                    ('image', '/camera/camera/image_raw'),
                    ('camera_info', '/camera/camera/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Node d'Apriltag
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='apriltag',
                remappings=[
                    ('/apriltag/image_rect', '/camera/image_rect'),
                    ('/camera/camera_info', '/camera/camera/camera_info')
                ],
                parameters=[apriltag_config_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    # Définition du node pour camera_pose_broadcaster
    control = Node(
        package='cobrapid_pi',
        executable='control',
        name='control_dirigeable',
        output='screen'
    )

    return LaunchDescription([
        device_arg,
        apriltag_container,
        control
    ])