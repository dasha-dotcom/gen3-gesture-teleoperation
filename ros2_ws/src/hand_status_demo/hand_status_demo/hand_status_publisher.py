import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HandStatusPublisher(Node):
	def __init__(self):
		super().__init__('hand_status_publisher')
		self.publisher_ = self.create_publisher(
			String,
			'hand_status',
			10
		)

		self.declare_parameter('publish_period', 1.0)
		publish_period = self.get_parameter('publish_period').value
		self.timer = self.create_timer(publish_period, self.publish_status)
	def publish_status(self):
		msg = String()
		msg.data = 'HAND_VISIBLE'
		self.publisher_.publish(msg)
		self.get_logger().info(f'Publishing: {msg.data}')
def main(args=None):
		rclpy.init(args=args)

		node = HandStatusPublisher()

		rclpy.spin(node)

		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
		main()
