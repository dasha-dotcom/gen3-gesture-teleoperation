from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='hand_status_demo',
			executable='hand_position_publisher',
			parameters=[
				{'publish_period': 0.25}
			]
		),
		Node(
			package='hand_status_demo',
			executable='hand_position_subscriber'
		)
	])
