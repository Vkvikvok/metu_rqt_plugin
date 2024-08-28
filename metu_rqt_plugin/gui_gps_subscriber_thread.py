import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.executors import MultiThreadedExecutor
        
class GPSSubscriberThread(QThread):
    gps_signal = pyqtSignal(float, float)  # PyQt5'te veri iletmek için sinyal

    def __init__(self, context, map_width, map_height): 
        super(GPSSubscriberThread, self).__init__()
        self.context = context
        
        self.node = None
        self.executor = None
        

        print("GPS Subscriber Thread started")

        # Harita hakkındaki veriler burada güncellenip öyle kullanılacak
        self.map_width = map_width # Kullanılan harita fotoğrafının piksel değerinden genişliği ve uzunluğu
        self.map_height = map_height
        # Burada enlemleri ters tanımlamamın sebebi piksellerin de koordinat sisteminin gps enlemine göre ters olması
        self.max_lat = -50 # Haritanın sol üst köşesinin derece cinsinden enlemi(Kuzey negatif, Güney pozitif)
        self.min_lat = -40 # Haritanın sağ alt köşesinin derece cinsinden enlemi
        self.min_lon = 20 # Haritanın sol üst köşesinin derece cinsinden boylamı(Doğu pozitif, Batı negatif)
        self.max_lon = 35 # Haritanın sağ alt köşesinin derece cinsinden boylamı

    def run(self):
        try:
            
            rclpy.init(context = self.context)
            

            # ROS2 düğümünü doğrudan iş parçacığı içinde oluşturun
            self.node = Node('gui_gps_subscriber', context=self.context)
            
            # Kapanma esnasında sıkıntı çıkmaması için "MultiThreadedExecutor" kullanılıyor
            self.executor = MultiThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            

            # Abonelik oluşturun
            self.subscription = self.node.create_subscription(
                NavSatFix,
                'gps_topic',
                self.listener_callback,
                10
            )

            # ROS2 düğümünü döngüde çalıştırın
            #rclpy.spin(self.node)
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        
        except Exception as e:
            print(f"There is an error in gps subscriber thread:{e}")
        
        
        finally:
            self.stop()

    def gps_to_pixel(self, latitude, longitude, map_width, map_height, max_lat, min_lat, max_lon, min_lon):
        x = (longitude - min_lon) * map_width / (max_lon - min_lon)
        # Aşağıda enlemler negatif alındığından dolayı burada da "min_lat" ile "max_lat" değerlerinin yerlerini değiştirerek kullandım
        y = (latitude + max_lat) * map_height / (max_lat - min_lat)         
        return x, y
    
    def stop(self):
        try:
        # Düğüm durdurulmadan önce rclpy'yi doğru şekilde kapatın
            if self.executor is not None:
                self.executor.shutdown()
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():    
                rclpy.shutdown()
            self.quit()

        except Exception as e:
            print("There is an error in closing gps subscriber thread:{e}")

    def listener_callback(self, msg):
        
        # Burada GPS verilerini harita piksellerine dönüştürüp arayüzde güncelleyebilirsiniz
        x_pixel, y_pixel = self.gps_to_pixel(msg.latitude, msg.longitude, self.map_width,
                                             self.map_height, self.max_lat, self.min_lat, self.max_lon, self.min_lon)
        print(f"GPS verileri piksellere dönüştürüldü: {x_pixel}, {y_pixel}")
        self.gps_signal.emit(x_pixel, y_pixel)
