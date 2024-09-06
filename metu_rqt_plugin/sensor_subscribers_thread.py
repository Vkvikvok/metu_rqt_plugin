import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.msg import Float64

class SensorSubscribersThread(QThread):
    flamable_gas_signal = pyqtSignal(int, float) 
    methane_gas_signal = pyqtSignal(int, float)
    carbon_mono_signal = pyqtSignal(int, float)
    env_temp_signal = pyqtSignal(int, float)
    env_pres_signal = pyqtSignal(int, float)
    env_humidity_signal = pyqtSignal(int, float)

    def __init__(self, context):
        super(SensorSubscribersThread, self).__init__()
        self.context = context

        self.flamable_gas_node = None
        self.methane_gas_node = None
        self.carbon_mono_node = None
        self.env_temp_node = None
        self.env_pres_node = None
        self.env_humidity_node = None
        self.executor = None

        

    def run(self):
        try:
            # ROS2'yi başlatın
            rclpy.init(context = self.context)
            
            # Düğümleri oluştur
            self.flamable_gas_node = Node("flamable_gas_subscriber", context=self.context)
            self.methane_gas_node = Node("methane_gas_subscriber", context=self.context)
            self.carbon_mono_node = Node("carbon_monoxide_subscriber", context=self.context)
            self.env_temp_node = Node("env_temp_subscriber", context=self.context)
            self.env_pres_node = Node("env_pressure_subscriber", context=self.context)
            self.env_humidity_node = Node("env_humidity_subscriber", context=self.context)

            # Daha optimize bir çalışma için "MultiThreadedExecutor" başlatılıyor ve içine gerekli düğümler atanıyor
            self.executor = MultiThreadedExecutor(context=self.context)
            self.executor.add_node(self.flamable_gas_node)
            self.executor.add_node(self.methane_gas_node)
            self.executor.add_node(self.carbon_mono_node)
            self.executor.add_node(self.env_temp_node)
            self.executor.add_node(self.env_pres_node)
            self.executor.add_node(self.env_humidity_node)

            # Abonelikleri oluşturun
            self.flamable_gas_subscription = self.flamable_gas_node.create_subscription(
                Float64, "flamable_gas", self.flamable_gas_callback, 10
            )
            self.methane_gas_subscription = self.methane_gas_node.create_subscription(
                Float64, "methane_gas", self.methane_gas_callback, 10
            )
            self.carbon_mono_subscription = self.carbon_mono_node.create_subscription(
                Float64, "carbon_mono", self.carbon_mono_callback, 10
            )
            self.env_temp_subscription = self.env_temp_node.create_subscription(
                Float64, "env_temp", self.env_temp_callback, 10
            )
            self.env_pres_subscription = self.env_pres_node.create_subscription(
                Float64, "env_pres", self.env_pres_callback, 10
            )
            self.env_humidity_subscription = self.env_humidity_node.create_subscription(
                Float64, "env_humidity", self.env_humidity_callback, 10
            )

            self.executor.spin()
            print("Executor başlatıldı")
        except Exception as e:
            print(f"There is an error in sensor subscribers thread:{e}")

        finally:
            self.stop()

    def flamable_gas_callback(self, msg):
        data = msg.data
        self.flamable_gas_signal.emit(0, data)
        print("Flamable gas data emitted")

    def methane_gas_callback(self, msg):
        data = msg.data
        self.methane_gas_signal.emit(1, data)
        print("Methane gas data emitted")

    def carbon_mono_callback(self, msg):
        data = msg.data
        self.carbon_mono_signal.emit(2, data)

    def env_temp_callback(self, msg):
        data = msg.data
        self.env_temp_signal.emit(3, data)

    def env_pres_callback(self, msg):
        data = msg.data
        self.env_pres_signal.emit(4, data)

    def env_humidity_callback(self, msg):
        data = msg.data
        self.env_humidity_signal.emit(5, data)

    def stop(self):
        try:
            if self.executor is not None:
                self.executor.shutdown()
            if self.flamable_gas_node is not None:
                self.flamable_gas_node.destroy_node()
            if self.methane_gas_node is not None:
                self.methane_gas_node.destroy_node()
            if self.carbon_mono_node is not None:
                self.carbon_mono_node.destroy_node()
            if self.env_temp_node is not None:
                self.env_temp_node.destroy_node()
            if self.env_pres_node is not None:
                self.env_pres_node.destroy_node()
            if self.env_humidity_node is not None:
                self.env_humidity_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.quit()
        
        except Exception as e:
            print(f"There is an error in closing sensor subscribers thread:{e}")
