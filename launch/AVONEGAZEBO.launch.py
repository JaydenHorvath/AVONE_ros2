import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro
import subprocess

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Locate the Xacro file
    pkg_path = get_package_share_directory('AVONE_description')
    xacro_file = os.path.join(pkg_path, 'AVONE', 'robot.urdf.xacro')

    # Process the Xacro file to URDF for robot_state_publisher (RViz side)
    urdf_for_rviz = xacro.process_file(xacro_file, mappings={'use_gazebo': 'false'}).toxml()

    # Process the Xacro file again for Gazebo (with Gazebo-style mesh paths)
    urdf_for_gazebo = xacro.process_file(xacro_file, mappings={'use_gazebo': 'true'}).toxml()

    # Save URDF for Gazebo to a temp file
    urdf_temp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    urdf_temp.write(urdf_for_gazebo.encode())
    urdf_temp.close()

    # Convert to SDF for Gazebo
    sdf_temp = tempfile.NamedTemporaryFile(delete=False, suffix=".sdf")
    sdf_output = subprocess.check_output(['ign', 'sdf', '-p', urdf_temp.name])
    sdf_temp.write(sdf_output)
    sdf_temp.close()

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': urdf_for_rviz
        }]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # TF bridge only (no /robot_description bridge!)
    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
        ],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_robot = ExecuteProcess(
        cmd=[
            'ign', 'service', '-s', '/world/empty/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'sdf_filename: "{sdf_temp.name}", name: "robot", pose: {{ position: {{ z: 1.0 }} }}'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        joint_state_publisher_gui_node,
        rsp_node,
        bridge_node,
        spawn_robot
    ])
