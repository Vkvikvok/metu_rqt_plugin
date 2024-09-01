from PyQt5.QtCore import QObject, QThread, pyqtSignal, QTimer
from rclpy.executors import SingleThreadedExecutor
import rclpy
from rclpy.node import Node

class TopicListenerThread(QThread):
    topic_list_signal = pyqtSignal(list)  # PyQt5'te veri iletmek için sinyal        
    
    def __init__(self, context):
        super(TopicListenerThread, self).__init__()

        self.context = context
        self.node = None
        self.executor = None

        # Zamanlayıcı oluşturarak sinyalin düzenli aralıklarla gönderilmesini sağlıyoruz
        self.timer = QTimer()
        self.timer.timeout.connect(self.listen_topics)
        self.timer.start(2000)  # 2000 milisaniye (2 saniye) aralıklarla kontrol fonksiyonunu çalıştırır

        print("Topic Listener Thread started")

    def run(self):
        try:
            rclpy.init(context = self.context)

            # ROS2 düğümünü başlat
            self.node = Node("topic_listener", context = self.context)
            print("Node başlatıldı")

            # Kapanma esanısında sıkıntı çıkmaması için "MultiThreadedExecutor" kullanıyoruz
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)

            self.executor.spin()

        except Exception as e:
            print(f"There is an error in topic listener:{e}")

        finally:
            self.stop()

    def stop(self):
        try:
        # Düğüm durdurulmadan önce rclpy'ı doğru şekilde kapatın
            self.timer.stop()
            if self.executor is not None:
                self.executor.shutdown(),
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.quit()
        
        except Exception as e:
            print(f"There is an error in closing topic listener thread:{e}")

    def listen_topics(self):
        topic_list = self.node.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_list]

        self.topic_list_signal.emit(topic_names)