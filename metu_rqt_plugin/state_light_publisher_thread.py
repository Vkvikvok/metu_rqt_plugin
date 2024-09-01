import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread
from rclpy.executors import SingleThreadedExecutor
from example_interfaces.msg import String

class StateLightNode(Node):
    def __init__(self, context):
        super().__init__('state_light_node', context=context)
        self.publisher_ = self.create_publisher(String, 'state_light', 10)

    def publish_message(self, value):
        msg = String()
        msg.data = value
        self.publisher_.publish(msg)

class StateLightPublisherThread(QThread):
    def __init__(self, context):
        super(StateLightPublisherThread, self).__init__()
        self.context = context
        
        self.state_light_node = None

    def run(self):
        try:
            # ROS2'yi başlatın
            rclpy.init( context= self.context) 

            # Düğümü oluştur
            self.state_light_node = StateLightNode(self.context)

            # Executor oluştur
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.state_light_node)
            
            self.executor.spin()
            
        except Exception as e:
            print(f"There is an error in state light publishers thread:{e}")

        finally:
            self.stop()  

    def publish_state_light_message(self, message):
        self.state_light_node.publish_message(message)  

    def stop(self):
        try:
            if self.executor is not None:
                self.executor.shutdown()
            if self.state_light_node is not None:
                self.state_light_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.quit()

        except Exception as e:
            print(f"There is an error in closing state light publisher thread:{e}")
