from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Launch TurtleBot3 Gazebo world
        DeclareLaunchArgument('world', default_value='/path/to/your/world/wheat_field.world'),
        
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            arguments=['$(arg world)']
        ),
        
        # Spawn four TurtleBot3 robots with different namespaces
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot_1',
            output='screen',
            arguments=['-file', '$(find turtlebot3_description)/models/turtlebot3_burger/model.sdf', '-robot_namespace', 'turtlebot_1', '-x', '0.0', '-y', '0.0', '-z', '0.0']
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot_2',
            output='screen',
            arguments=['-file', '$(find turtlebot3_description)/models/turtlebot3_burger/model.sdf', '-robot_namespace', 'turtlebot_2', '-x', '2.0', '-y', '0.0', '-z', '0.0']
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot_3',
            output='screen',
            arguments=['-file', '$(find turtlebot3_description)/models/turtlebot3_burger/model.sdf', '-robot_namespace', 'turtlebot_3', '-x', '4.0', '-y', '0.0', '-z', '0.0']
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot_4',
            output='screen',
            arguments=['-file', '$(find turtlebot3_description)/models/turtlebot3_burger/model.sdf', '-robot_namespace', 'turtlebot_4', '-x', '6.0', '-y', '0.0', '-z', '0.0']
        ),
        
        # Start your custom nodes for each TurtleBot with different namespaces
        Node(
            package='detection_robot',
            executable='detection_robot.py',
            name='detection_robot_1',
            output='screen',
            parameters=[{'robot_name': 'turtlebot_1'}]
        ),
        
        Node(
            package='intervention_robot',
            executable='intervention_robot.py',
            name='intervention_robot_1',
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
        
        Node(
            package='intervention_robot',
            executable='intervention_robot.py',
            name='intervention_robot_2',
            output='screen',
            parameters=[{'robot_name': 'turtlebot_2'}]
        ),
         
    ])

