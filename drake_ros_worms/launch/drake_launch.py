import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    drake_ros_worms_share = get_package_share_directory('drake_ros_worms')

    # Manually extract scene_file_path from params (model_interface is not a node)
    with open(os.path.join(drake_ros_worms_share, 'config', 'params.yaml'), 'r') as file:
        params = yaml.safe_load(file)
        if "scene_file_path" not in params: 
            raise RuntimeError("Must set scene_file_path in params.yaml")
        scene_file_path = params["scene_file_path"]
        simulator_frequency = str(params.get("simulator_frequency"))

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d' + os.path.join(drake_ros_worms_share, 'config', 'drake_default.rviz')
            ]
        ),
        Node(
            package='drake_ros_worms',
            executable='model_interface',
            name='drake',
            arguments=[
                "--scene-file-path", scene_file_path,
                "--simulator-frequency", simulator_frequency
            ]
        )
    ])