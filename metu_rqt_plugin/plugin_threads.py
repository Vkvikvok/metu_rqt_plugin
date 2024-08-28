from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# GPS bilgilerinin publisher dan alınıp arayüze aktarılmasını sağlayan iş parçacığı sınıfı
class CameraSubscriberThread(QThread):
    gps_signal = pyqtSignal(float, float)  # PyQt5'te veri iletmek için sinyal

    def __init__(self, context):
        super().__init__()
    
        self._running = False
        self.context = context
        self.node = None

    # İş parçacığı çalıştırıldığında parçacığın yapması gerekenlerin tanımlandığı fonksiyon
    def run(self):
        try:
            self._running = True
            if self.node is None:
                self.node = Node('qt_ros_node', context=self.context)
                #self.node = rclpy.create_node('camera_subscriber_thread')
                self.subscription = self.node.create_subscription(
                    Image,
                    'camera/image_raw',
                    self.listener_callback,
                    10
                )

            # Subscriber düğümünün çalıştırılmasını sağlar
            while self._running:
                rclpy.spin_once(self.node)
                self.msleep(10)
                
        except Exception as e:
            print(f"There is an error in running thread:{e}")
            #self.get_logger().info(f"There is an error in running thread:{e}")

    # Görüntülerin işlenmesi ve arayüze çekilmesini sağlayan callback
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Burada istenilen görüntü işleme yapılabilir

            self.image_update.emit(rgb_image)
            
        except Exception as e:  
            print(f"There is an error in camera listener:{e}")          

    # Kamera arayüz içerisinde kapatıldığında düğümü kapatmak yerine beklemeye alır
    def stop(self):
        try:
            self._running = False # Burada callback içerisindeki while döngüsü durdurulur
            
        except Exception as e:
            #self.get_logger().info(f"Kamera işlemcisinin durdurulmasında sıkıntı çıktı:{e}")
            print(f"Kamera işlemcisinin durdurulmasında sıkıntı çıktı:{e}")
            

    # Arayüz kapatıldığında düğümün çalışmasını da kapatır
    def shutdown(self):
        if self.node:
            self.node.destroy_node()
            self.node = None
            rclpy.shutdown()




#########################################3
# QWidget init
# Roverın konumunu gösteren markerın tanımlanması
        self.rover_marker = None

# Roverın konumu güncelleniyor
    def update_position(self, x_pixel, y_pixel):
        try:
            if self.rover_marker is None:
                self.rover_marker = QGraphicsEllipseItem(x_pixel, y_pixel, 10, 10)  # Marker size
                self.rover_marker.setBrush(Qt.blue)
                self.scene.addItem(self.rover_marker)
                print("Rover markerı oluşturuldu")
            else:
                self.rover_marker.setPos(QPointF(x_pixel, y_pixel))
                print(f"Roverın konumunu değiştirildi:{x_pixel},{y_pixel}")
            print(f"Harita pozisyonu alındı:{x_pixel},{y_pixel}")
        except Exception as e:
            print(f"There is an error in updating rover marker position:{e}")
