from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from python_qt_binding.QtCore import Qt
import rclpy
from rclpy.context import Context
from rqt_gui.main import Main
import sys
import os
# Özelleştirilmiş elemanlar
from .state_light_publisher_thread import StateLightPublisherThread

class StateLight(Plugin):
    def __init__(self, context):
        super(StateLight, self).__init__(context)
        self.setObjectName('StateLight')

        # Ana widget'ı oluştur ve ayarlarını yap
        self._widget = MyWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # Widget'ı ve iş parçacığını durdur
        self._widget.shutdown()

class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        # Eklenti arayüzü tasarlanıyor
        layout = QHBoxLayout()
        light_layout = QVBoxLayout()
        button_layout = QVBoxLayout()


        self.red_light = QLabel(self)
        self.yellow_light = QLabel(self)
        self.green_light = QLabel(self)

        self.set_light_style(self.red_light, "grey")
        self.set_light_style(self.yellow_light, "grey")
        self.set_light_style(self.green_light, "grey")

        light_layout.addWidget(self.red_light)
        light_layout.addWidget(self.yellow_light)
        light_layout.addWidget(self.green_light)

        red_button = QPushButton('Red', self)
        yellow_button = QPushButton('Yellow', self)
        green_button = QPushButton('Green', self)

        button_layout.addWidget(red_button)
        button_layout.addWidget(yellow_button)
        button_layout.addWidget(green_button)

        layout.addLayout(light_layout)
        layout.addLayout(button_layout)

        self.setLayout(layout)

        # Buton ve publisher bağlantıları burada yapılıyor
        try:
            self.context = Context()
            self.state_light_publisher_thread = StateLightPublisherThread(self.context)
            self.state_light_publisher_thread.start()

            red_button.clicked.connect(lambda: self.set_traffic_light('red'))
            yellow_button.clicked.connect(lambda: self.set_traffic_light('yellow'))
            green_button.clicked.connect(lambda: self.set_traffic_light('green'))

        except Exception as e:
            print(f"There is an error in controlling state lights:{e}")

    def set_light_style(self, light_label, color):
        light_label.setFixedSize(100, 100)
        light_label.setStyleSheet(f"background-color: {color}; border-radius: 50%;")

    def set_traffic_light(self, color):
        # Reset all lights to grey
        self.set_light_style(self.red_light, "grey")
        self.set_light_style(self.yellow_light, "grey")
        self.set_light_style(self.green_light, "grey")

        # Set the selected light to the specified color
        if color == 'red':
            self.set_light_style(self.red_light, "red")
        elif color == 'yellow':
            self.set_light_style(self.yellow_light, "yellow")
        elif color == 'green':
            self.set_light_style(self.green_light, "green")

        self.state_light_publisher_thread.publish_state_light_message(color)

    def shutdown(self):
        # İş parçacığını durdur
        self.state_light_publisher_thread.stop()

def main():
    sys.exit(Main().main(sys.argv, standalone="metu_rqt_plugin.state_light_plugin"))

if __name__ == "__main__":
    main()
