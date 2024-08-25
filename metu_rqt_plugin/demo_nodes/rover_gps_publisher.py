import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # GPS verileri için kullanılan standart mesaj tipi

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_topic', 10)
        timer_period = 1.0  # Her saniye bir veri yayınla
        self.timer = self.create_timer(timer_period, self.publish_gps_data)

   #### Buradaya sonrasında anlık konum hareketini gözlemlemek için rastgele seri veri üreten bir algoritma yazılabilir#####
    def publish_gps_data(self):
        msg = NavSatFix()
        msg.latitude = 40.748817  # Örneğin bir koordinat (Enlem)
        msg.longitude = -73.985428  # Örneğin bir koordinat (Boylam)
        msg.altitude = 10.0  # Yükseklik
        self.publisher_.publish(msg)
        self.get_logger().info(f'GPS verisi yayınlandı: {msg.latitude}, {msg.longitude}, {msg.altitude}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
