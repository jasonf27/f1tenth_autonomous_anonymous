# turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='pac1', executable='talker.py', parameters = [
            	{'v': 1.0},
            	{'d': 2.0}
            ]
    	),
        launch_ros.actions.Node(
            package='pac1', executable='relay.py'
        )
    ])
