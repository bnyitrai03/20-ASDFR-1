from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    adapter_node = Node(
        package='relbot_adapter',
        executable='relbot_adapter',
        name='relbot_adapter',
        parameters=[
            {"use_twist_cmd": False}, 
            {"max_speed_rads": 30.0},  
            {"robotmode": "sim"}     
        ]
    )

    simulator_node = Node(
        package='relbot_simulator',
        executable='relbot_simulator',
        name='relbot_sim',
        remappings=[
            ('/input/motor_cmd', '/output/motor_cmd')
        ]
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='robotturtle'
    )

    sequence_node = Node(
        package='assignment1_1',             
        executable='sequence_controller', 
        name='seq_ctrl'
    )

    position_node = Node(
        package='assignment1_1',
        executable='position_indicator',
        name='pos_indcator'
    )

    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        name='cam2image',
        parameters=[
            {"reliability": "reliable"} 
        ]
    )

    return LaunchDescription([
        adapter_node,
        simulator_node,
        turtlesim_node,
        sequence_node,
        camera_node,
        position_node
    ])