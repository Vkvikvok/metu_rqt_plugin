from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtWidgets import QApplication, QWidget, QLineEdit, QListView, QVBoxLayout, QPushButton
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem, QBrush
import rclpy
from rclpy.context import Context
from rqt_gui.main import Main
import sys
import os
# Özelleştirilmiş elemanlar
from .topic_controller_ui import Ui_TopicController
from .custom_plugin_widgets import CustomListView
from .topic_listener_thread import TopicListenerThread

class TopicController(Plugin):
    def __init__(self, context):
        super(TopicController, self).__init__(context)
        self.setObjectName('TopicController')

        # Ana widget'ı oluştur ve ayarlarını yap
        self._widget = MyWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        try:
            # Dönüştürülmüş UI'yi yükle
            self.ui = Ui_TopicController()  # Bu sınıf, pyuic5 tarafından oluşturulan sınıftır
            self.ui.setupUi(self)  # Arayüz öğelerini bu widget üzerine kur
            self.topic_input = self.ui.lineEdit
            self.add_button = self.ui.pushButton

            # Liste elemanı için ayarlamalar
            self.model = QStandardItemModel(self)
            self.customListView = CustomListView(self.model)

            # Layout'taki eski QGraphicsView'i kaldır ve yeni CustomGraphicsView'i ekle
            layout = self.ui.listView.parentWidget().layout() 
            layout.replaceWidget(self.ui.listView, self.customListView)
            self.ui.listView.deleteLater()  # Orijinal QGraphicsView'i temizle

            # Topic adını gireceğimiz kısmın ayarları
            self.topic_input.setPlaceholderText("Enter ROS topic name")

            # QPushButton: Topic adını listeye eklemek için
            self.add_button.setText("Add Topic")
            self.add_button.clicked.connect(self.add_topic)
            self.add_button.setShortcut("Enter")

        except Exception as e:
            print(f"There is an error in initialization of ui: {e}")

        # Topic listesinin çekilebilmesi için iş parçacığını başlatıyoruz
        try:
            self.context = Context()
            self.topic_listener_thread = TopicListenerThread(self.context)
            self.topic_listener_thread.topic_list_signal.connect(self.check_topics) # Sinyali fonksiyona bağlıyoruz
            self.topic_listener_thread.start()

        except Exception as e:
            print(f"There is an error in starting topic listener thread: {e}")

    def add_topic(self):
        # QLineEdit'e girilen text'i alıyoruz
        topic_name = self.topic_input.text().strip()
        
        if topic_name:  # Eğer text boş değilse

            edited_topic_name = topic_name.strip()
            if not edited_topic_name.startswith("/"):
                edited_topic_name = "/" + edited_topic_name
            # Topic adını listeye ekliyoruz
            item = QStandardItem(edited_topic_name)
            
            # Modeli güncelliyoruz
            self.model.appendRow(item)           
            
            # Girdi alanını temizliyoruz
            self.topic_input.clear()
    
    # Topiclerin aktifliğinin sürekli şekilde kontrol edildiği fonksiyon
    def check_topics(self, topic_list):
        for row in range(self.model.rowCount()):
            item = self.model.item(row)
            if item is not None:
                topic_name = item.text()
                if topic_name in topic_list:
                    item.setBackground(QBrush(Qt.green))
                else:
                    item.setBackground(QBrush(Qt.red))

    def shutdown(self):
        # İş parçacığını durdur
        self.topic_listener_thread.stop()

def main():
    sys.exit(Main().main(sys.argv, standalone="metu_rqt_plugin.topic_controller_plugin"))

if __name__ == "__main__":
    main()
