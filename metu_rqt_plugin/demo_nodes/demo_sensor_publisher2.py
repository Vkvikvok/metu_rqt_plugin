import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64
import random

class DemoSensorPublisher(Node):

    def __init__(self):
        super().__init__('demo_sensor_publisher2')
        self.publisher_ = self.create_publisher(Float64, 'methane_gas', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_random_float)
        self.get_logger().info('RandomFloatPublisher node has been started.')

    def publish_random_float(self):
        msg = Float64()
        msg.data = random.uniform(0.0, 100.0)  # Random float between 0.0 and 100.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DemoSensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
