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
from metu_rqt_plugin.ui.topic_controller_ui import Ui_TopicController
from metu_rqt_plugin.custom_plugin_widgets import CustomListView

class TopicController(Plugin):
    def __init__(self, context):
        super(TopicController, self).__init__(context)
        self.setObjectName('TopicController')


        # Rclpy'ı başlat ve context i çek(Threading için)
        self.context = Context()

        rclpy.init(context=self.context)

        # Topicleri dinleyebilecek bir node oluşturuyoruz
        self.node = rclpy.create_node('topic_listener_plugin_node')


        # Ana widget'ı oluştur ve ayarlarını yap
        self._widget = MyWidget(self.context, self.node)
        context.add_widget(self._widget)

        # ROS2 context'i kapat
        rclpy.shutdown(context=self.context)

class MyWidget(QWidget):
    def __init__(self, context, node):
        super(MyWidget, self).__init__()

        self.context = context # ROS2 contextinin çekilmesi

        self.node = node
        
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

        # Zamanlayıcıyı oluşturuyoruz
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_topics)
        self.timer.start(2000)  # 2000 milisaniye (2 saniye) aralıklarla kontrol fonksiyonunu çalıştırır

    def add_topic(self):
        # QLineEdit'e girilen text'i alıyoruz
        topic_name = self.topic_input.text().strip()
        
        if topic_name:  # Eğer text boş değilse

            edited_topic_name = topic_name.strip()
            if not edited_topic_name.startswith("/"):
                edited_topic_name = "/" + edited_topic_name
            # Topic adını listeye ekliyoruz
            item = QStandardItem(edited_topic_name)
            
            # Topic adı ilk girildiğinde aktif olup olmadığına göre renklendirme
            if edited_topic_name in self.get_all_topics():
                item.setBackground(QBrush(Qt.green))
            else:
                item.setBackground(QBrush(Qt.red))

            # Modeli güncelliyoruz
            self.model.appendRow(item)           
            
            # Girdi alanını temizliyoruz
            self.topic_input.clear()

    # ROS 2'den topic'leri alıyoruz
    def get_all_topics(self):
        topic_list = self.node.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_list]

        return topic_names
    
    # Topiclerin aktifliğinin sürekli şekilde kontrol edildiği fonksiyon
    def check_topics(self):
        for row in range(self.model.rowCount()):
            item = self.model.item(row)
            if item is not None:
                topic_name = item.text()
                if topic_name in self.get_all_topics():
                    item.setBackground(QBrush(Qt.green))
                else:
                    item.setBackground(QBrush(Qt.red))

def main():
    sys.exit(Main().main(sys.argv, standalone="metu_rqt_plugin.topic_controller_plugin"))

if __name__ == "__main__":
    main()
