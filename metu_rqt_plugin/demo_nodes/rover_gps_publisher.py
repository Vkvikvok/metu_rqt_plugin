import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # GPS verileri için kullanılan standart mesaj tipi

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_topic', 10)
        timer_period = 1.0  # Her saniye bir veri yayınla
        self.timer = self.create_timer(timer_period, self.publish_gps_data)

        ###################
        self.latitude = 50.065984 # Başlangıç Noktası
        self.longitude = 19.913646  # Başlangıç Noktası
        self.lat_step = 0 # Enlemde hareket (Kuzeyden Güneye)
        self.lon_step = 0  # Boylamda hareket (Batıdan Doğuya)
        ###################

        self.get_logger().info("Rover GPS publisher has been started")

   #### Buradaya sonrasında anlık konum hareketini gözlemlemek için rastgele seri veri üreten bir algoritma yazılabilir#####
    def publish_gps_data(self):
        msg = NavSatFix()
        ########################
        msg.latitude = self.latitude  # Örneğin bir koordinat (Enlem)
        msg.longitude = self.longitude  # Örneğin bir koordinat (Boylam)
        #########################
        msg.altitude = 10.0  # Yükseklik (opsiyonel)
        self.publisher_.publish(msg)
        self.get_logger().info(f'GPS verisi yayınlandı: {msg.latitude}, {msg.longitude}, {msg.altitude}')

        #######################
        # GPS Değerlerini Güncelle
        self.latitude += self.lat_step
        self.longitude += self.lon_step

        # Hareket sınırları kontrolü
        #if self.latitude < -50.066325 or self.longitude > 19.913640:
         #   self.lat_step = -self.lat_step  # Geri dönüş
        #    self.lon_step = -self.lon_step  # Geri dönüş
        ########################

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
