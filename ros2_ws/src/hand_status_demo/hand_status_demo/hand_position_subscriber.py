import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class HandPositionSubscriber(Node):
	def __init__(self):
        	super().__init__('hand_position_subscriber')

        	self.subscription = self.create_subscription(
			 PointStamped,
            		'hand_position',
            		self.read_position,
           		10
        	)

	def read_position(self, msg):
		self.get_logger().info(
			f'Received hand position: '
			f'x={msg.point.x}, y={msg.point.y}, z={msg.point.z}, '
			f'frame={msg.header.frame_id}'
		)

def main(args=None):
	rclpy.init(args=args)

	node = HandPositionSubscriber()

	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
