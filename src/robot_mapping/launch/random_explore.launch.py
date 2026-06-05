from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '/home/anshul/robot_mapping_ws/src/random_world.sdf'],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch',
                    'turtlebot3_gazebo',
                    'spawn_turtlebot3.launch.py'
                ],
                output='screen'
            )
        ]
    )

    robot_state_pub = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch',
                    'turtlebot3_gazebo',
                    'robot_state_publisher.launch.py'
                ],
                output='screen'
            )
        ]
    )

    slam = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch',
                    'slam_toolbox',
                    'online_async_launch.py'
                ],
                output='screen'
            )
        ]
    )

    rviz = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['rviz2'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_pub,
        slam,
        rviz
    ])
