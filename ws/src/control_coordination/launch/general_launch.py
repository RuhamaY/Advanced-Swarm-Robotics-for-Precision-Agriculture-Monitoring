from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'world' argument for the Gazebo world file
        DeclareLaunchArgument('world', default_value='/home/ros/ws/worlds/wheat_field.world', description='Path to the Gazebo world file'),
        
        # Launch Gazebo server
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            arguments=['--world', '$(arg world)']
        ),
        
        # Launch Gazebo client (optional, if you want GUI)
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_client',
            output='screen'
        ),
        
        # Spawn TurtleBot3 robots in the Gazebo environment
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot_1',
            output='screen',
            arguments=[
                '-file', '$(find turtlebot3_description)/models/turtlebot3_burger/model.sdf',
                '-robot_namespace', 'turtlebot_1',
                '-x', '0.0', '-y', '0.0', '-z', '0.0'
            ]
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot_2',
            output='screen',
            arguments=[
                '-file', '$(find turtlebot3_description)/models/turtlebot3_burger/model.sdf',
                '-robot_namespace', 'turtlebot_2',
                '-x', '2.0', '-y', '0.0', '-z', '0.0'
            ]
        ),
        
        # Start custom detection nodes for TurtleBot 1 and 2
        Node(
            package='detection_robot',
            executable='detection_robot.py',
            name='detection_robot_1',
            output='screen',
            parameters=[{'robot_name': 'turtlebot_1'}]
        ),
        
        Node(
            package='detection_robot',
            executable='detection_robot.py',
            name='detection_robot_2',
            output='screen',
            parameters=[{'robot_name': 'turtlebot_2'}]
        ),

        # Start custom intervention nodes for TurtleBot 1 and 2 (namespace issue fixed)
        Node(
            package='intervention_robot',
            executable='intervention_robot.py',
            name='intervention_robot_1',
            output='screen',
            parameters=[{'robot_name': 'turtlebot_1'}]
        ),
        
        Node(
            package='intervention_robot',
            executable='intervention_robot.py',
            name='intervention_robot_2',
            output='screen',
            parameters=[{'robot_name': 'turtlebot_2'}]
        ),
    ])

