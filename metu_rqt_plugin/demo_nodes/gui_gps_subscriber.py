import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(NavSatFix, 'gps_topic', self.listener_callback, 10)
        self.subscription  # Prevent unused variable warning
        # Harita hakkındaki veriler burada güncellenip öyle kullanılacak
        self.map_width = None
        self.map_height = None
        self.origin_lat = None
        self.origin_lon = None
        self.scale = None

    def listener_callback(self, msg):
        self.get_logger().info(f'GPS verisi alındı: {msg.latitude}, {msg.longitude}, {msg.altitude}')
        # Burada GPS verilerini harita piksellerine dönüştürüp arayüzde güncelleyebilirsiniz
        x_pixel, y_pixel = self.gps_to_pixel(msg.latitude, msg.longitude, self.map_width,
                                             self.map_height, self.origin_lat, self.origin_lon, self.scale)


    def gps_to_pixel(self, latitude, longitude, map_width, map_height, origin_lat, origin_lon, scale):
        # Enlem/boylam farklarını hesapla
        delta_lat = latitude - origin_lat
        delta_lon = longitude - origin_lon

        # Piksel koordinatlarını hesapla
        x_pixel = map_width / 2 + (delta_lon * scale)
        y_pixel = map_height / 2 - (delta_lat * scale)

        return int(x_pixel), int(y_pixel)

        

def main(args=None):
    rclpy.init(args=args)
    node = GPSSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
