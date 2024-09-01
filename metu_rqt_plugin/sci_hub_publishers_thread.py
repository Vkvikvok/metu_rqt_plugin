import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.msg import Int32

class DrillControlNode(Node):
    def __init__(self, context):
        super().__init__('drill_control_node', context=context)
        self.publisher_ = self.create_publisher(Int32, 'drill_control', 10)

    def publish_message(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)

class SciPlatControlNode(Node):
    def __init__(self, context):
        super().__init__('sci_plat_control_node', context=context)
        self.publisher_ = self.create_publisher(Int32, 'science_platform_control', 10)

    def publish_message(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)

class DrillHeadNode(Node):
    def __init__(self, context):
        super().__init__('drill_head_node', context=context)
        self.publisher_ = self.create_publisher(Int32, 'drill_head', 10)

    def publish_message(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)

class SciPlatRotationNode(Node):
    def __init__(self, context):
        super().__init__('sci_plat_rotation_node', context=context)
        self.publisher_ = self.create_publisher(Int32, 'container_rotate', 10)

    def publish_message(self, value):
        msg = Int32()
        msg.data = value
        self.publisher_.publish(msg)

class SciHubPublishersThread(QThread):
    def __init__(self, context):
        super(SciHubPublishersThread, self).__init__()
        self.context = context

        self.drill_control_node = None
        self.sci_plat_control_node = None
        self.drill_head_node = None
        self.sci_plat_rotation_node = None

    def run(self):
        try:
            # ROS2'yi başlatın
            rclpy.init( context= self.context) 

            # Düğümleri oluştur
            self.drill_control_node = DrillControlNode(context=self.context)
            self.sci_plat_control_node = SciPlatControlNode(context=self.context)
            self.drill_head_node = DrillHeadNode(context=self.context)
            self.sci_plat_rotation_node = SciPlatRotationNode(context=self.context)

            # Daha optimize bir çalışma için "MultiThreadedExecutor" başlatılıyor ve içine gerekli düğümler atanıyor
            self.executor = MultiThreadedExecutor(context=self.context)
            self.executor.add_node(self.drill_control_node)
            self.executor.add_node(self.sci_plat_control_node)
            self.executor.add_node(self.drill_head_node)
            self.executor.add_node(self.sci_plat_rotation_node)

            self.executor.spin()

        except Exception as e:
            print(f"There is an error in science hub publishers thread:{e}")

        finally:
            self.stop()

    def publish_drill_control_message(self, message):
        self.drill_control_node.publish_message(message)

    def publish_sci_plat_control_message(self, message):
        self.sci_plat_control_node.publish_message(message)

    def publish_drill_head_message(self, message):
        self.drill_head_node.publish_message(message)

    def publish_sci_plat_rotation_message(self, message):
        self.sci_plat_rotation_node.publish_message(message)

    def stop(self):
        try:
            if self.executor is not None:
                self.executor.shutdown()
            if self.drill_control_node is not None:
                self.drill_control_node.destroy_node()
            if self.sci_plat_control_node is not None:
                self.sci_plat_control_node.destroy_node()
            if self.drill_head_node is not None:
                self.drill_head_node.destroy_node()
            if self.sci_plat_rotation_node is not None:
                self.sci_plat_rotation_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.quit()

        except Exception as e:
            print(f"There is an error in closing science hub publishers thread:{e}")

