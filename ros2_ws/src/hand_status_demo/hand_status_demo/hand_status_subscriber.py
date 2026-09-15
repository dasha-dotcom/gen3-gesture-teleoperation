import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HandStatusSubscriber(Node):
	def __init__(self):
		super().__init__('hand_status_subscriber')
		
		self.subscription = self.create_subscription(
			String,
			'hand_status',
			self.read_message,
			10
		)
		
	def read_message(self, msg):
		self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
	rclpy.init(args=args)
	
	node = HandStatusSubscriber()

	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
