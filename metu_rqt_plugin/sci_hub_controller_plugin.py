from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtWidgets import QWidget, QPushButton, QLabel
import rclpy
from rclpy.context import Context
from rqt_gui.main import Main
import sys
import os
from example_interfaces.msg import Int32
# Özelleştirilmiş elemanlar
from metu_rqt_plugin.ui.sci_hub_controller_ui import SciHubControllerUi
from metu_rqt_plugin.threads.sci_hub_publishers_thread import SciHubPublishersThread


class SciHubController(Plugin):
    def __init__(self, context):
        super(SciHubController, self).__init__(context)
        self.setObjectName('SciHubController')

        # Ana widget'ı oluştur ve ayarlarını yap
        self._widget = MyWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # Widget'ı ve iş parçacığını durdur
        self._widget.shutdown()

class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        # Dönüştürülmüş UI'yi yükle
        self.ui = SciHubControllerUi()  # Bu sınıf, pyuic5 tarafından oluşturulan sınıftır
        self.ui.setupUi(self)  # Arayüz öğelerini bu widget üzerine kur

        # Science Hub Publishers Thread inin başlatılması
        try:
            self.context = Context()
            self.sci_hub_publisher_thread = SciHubPublishersThread(self.context)
            self.sci_hub_publisher_thread.start()

        except Exception as e:
            print(f"There is an error in starting sci hub publishers thread:{e}")

        # Butonlara fonksiyonlar atanması
        try:
            self.ui.drill_up_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_drill_control_message(1))
            self.ui.drill_stop_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_drill_control_message(0))
            self.ui.drill_down_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_drill_control_message(2))

            self.ui.sci_plat_up_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_sci_plat_control_message(1))
            self.ui.sci_plat_stop_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_sci_plat_control_message(0))
            self.ui.sci_plat_down_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_sci_plat_control_message(2))

            self.ui.drill_head_cw_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_drill_head_message(1))
            self.ui.drill_head_stop_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_drill_head_message(0))
            self.ui.drill_head_ccw_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_drill_head_message(2))

            self.ui.rotate_60_deg_cw_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_sci_plat_rotation_message(1))
            self.ui.rotate_60_deg_ccw_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_sci_plat_rotation_message(2))
            self.ui.rotate_1_step_cw_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_sci_plat_rotation_message(3))
            self.ui.rotate_1_step_ccw_button.clicked.connect(lambda: self.sci_hub_publisher_thread.publish_sci_plat_rotation_message(4))

        except Exception as e:
            print(f"There is an error in the sending button messages:{e}")

    def shutdown(self):
        # İş parçacığını durdur
        self.sci_hub_publisher_thread.stop()

def main():
    sys.exit(Main().main(sys.argv, standalone="metu_rqt_plugin.competition_map_plugin"))

if __name__ == "__main__":
    main()