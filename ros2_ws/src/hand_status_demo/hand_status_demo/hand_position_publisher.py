import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class HandPositionPublisher(Node):
	def __init__(self):
		super().__init__('hand_position_publisher')

		self.publisher_ = self.create_publisher(
			PointStamped,
			'hand_position',
			10
		)
		
		self.declare_parameter('publish_period', 1.0)
		publish_period = self.get_parameter('publish_period').value
		
		self.timer = self.create_timer(publish_period, self.publish_position)

	def publish_position(self):
		msg = PointStamped()
		msg.point.x = 0.20
		msg.point.y = -0.10
		msg.point.z = 0.45
		msg.header.frame_id = 'camera_frame'
		msg.header.stamp = self.get_clock().now().to_msg()
		self.publisher_.publish(msg)
		self.get_logger().info(
			f'Publishing hand position: '
			f'x={msg.point.x}, y={msg.point.y}, z={msg.point.z}, '
			f'frame={msg.header.frame_id}'
		)

def main(args=None):
	rclpy.init(args=args)

	node = HandPositionPublisher()

	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
