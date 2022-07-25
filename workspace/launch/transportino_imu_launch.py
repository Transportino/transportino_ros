from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            remappings=[
                ('/imu/data_raw', '/transportino/board/imu'),
                ('/imu/data', '/transportino/imu'),
            ],
            parameters=[
                {"use_mag": False}
            ]
        )
    ])