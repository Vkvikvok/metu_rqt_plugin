from rqt_gui_py.plugin import Plugin
from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtWidgets import QApplication, QWidget, QGraphicsScene, QGraphicsView, QLineEdit, QHeaderView, QTableView, QGraphicsEllipseItem
from python_qt_binding.QtGui import QDoubleValidator, QKeySequence, QImage, QPixmap, QBrush, QColor
from python_qt_binding.QtCore import Qt, QPointF
import rclpy
from rclpy.context import Context
# Özelleştirilmiş elemanlar
from .custom_plugin_widgets import CustomGraphicsView, MyTableModel, CustomTableView
from .competition_map_ui import CompetitionMapUi
from .gui_gps_subscriber_thread import GPSSubscriberThread

from rqt_gui.main import Main
import sys
import os

class CompetitionMap(Plugin):
    def __init__(self, context):
        super(CompetitionMap, self).__init__(context)
        self.setObjectName('CompetitionMap')

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
        self.ui = CompetitionMapUi()  # Bu sınıf, pyuic5 tarafından oluşturulan sınıftır
        self.ui.setupUi(self)  # Arayüz öğelerini bu widget üzerine kur

        # Yarışma alanının haritasını gösteren parça
        try:
            self.scene = QGraphicsScene()
            self.customCompetitionMap = CustomGraphicsView(self.scene, box_x=self.ui.send_x, box_y=self.ui.send_y)

            # Layout'taki eski QGraphicsView'i kaldır ve yeni CustomGraphicsView'i ekle
            layout = self.ui.competitionMap.parentWidget().layout() 
            layout.replaceWidget(self.ui.competitionMap, self.customCompetitionMap)
            self.ui.competitionMap.deleteLater()  # Orijinal QGraphicsView'i temizle

            # Görseli yükle
            self.base_dir = os.path.dirname(os.path.abspath(__file__))
            self.customCompetitionMap.load_image(os.path.join(self.base_dir,'new_map.jpg'))

        except Exception as e:
            print(f"There is an error in loading competition map: {e}")


        # Roverın gideceği konumların listesinin ayarlanması
        try:
            self.model = MyTableModel()
            self.locationList = CustomTableView()
            self.locationList.setModel(self.model) # Özelleştirilmiş "QTableView" elemanını çağırıyoruz

            # Layout'taki eski QTableView elemanını kaldır ve yeni CustomTableView elemanını ekle
            layout = self.ui.location_widgets_layout
            layout.replaceWidget(self.ui.location_list, self.locationList)
            self.ui.location_list.deleteLater()  # Orijinal QTableView'i temizle

            # Tablo sütunlarının arayüze daha uygun şekilde boyutlanmasını sağlar
            self.locationList.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

            # Tablodan bir hücre seçildiği zaman tüm satırın seçilmesini sağlar
            self.locationList.setSelectionMode(QTableView.SingleSelection)
            self.locationList.setSelectionBehavior(QTableView.SelectRows)

        except Exception as e:
            print(f"There is an error in location list: {e}")

        try:
        # x ve y verilerini alacağımız kutuların ayarlanması
            validator = QDoubleValidator() # Sadece sayısal değerlerin girilmesini sağlar(e hariç)
            self.ui.send_x.setValidator(validator)
            self.ui.send_y.setValidator(validator)
            
            # "Send Location" butonunun düzenlenmesi
            self.ui.send_location_button.setShortcut(QKeySequence("Enter")) # Enter kısayol olarak ayarlandı
            self.ui.send_location_button.clicked.connect(self.sendLocation) # Gerekli fonksiyon atandı
            
            # "Go/Cancel" butonunun düzenlenmesi
            self.ui.go_cancel_button.clicked.connect(self.toggle_row_highlight)
            self.highlight = False

        except Exception as e:
            print(f"There is an error in coordination boxes:{e}")

        # Harita üzerinde rover ve gidilecek konumların işlenmesi ve gösterilmesi
        try:
            # GPS sinyali ile roverın konumunu almak için GPSSubscriberThread başlatılıyor
            self.context = Context()
            self.gps_thread = GPSSubscriberThread(self.context, 
                                                  self.customCompetitionMap.map_item.pixmap().width(), 
                                                  self.customCompetitionMap.map_item.pixmap().height())
            self.gps_thread.gps_signal.connect(self.update_position)  # Sinyali bağla
            self.gps_thread.start()

            # Roverın konumunu gösterecek markerın tanımlanması
            self.rover_marker = None

            # Tablonun değişme sinyali alınıp harita üzerinden noktalar yenileniyor
            self.model.dataChanged.connect(self.update_graphics_view)

        except Exception as e:
            print(f"There is an error in presentation of location points and rover location:{e}")
     

    # Harita üzerindeki konum noktaları güncelleniyor
    def update_graphics_view(self):
        print("Veriler değiştirildi")
        data = [self.model._data[i] for i in range(self.model.rowCount())]
        self.customCompetitionMap.update_points(data)

    # Roverın konumu güncelleniyor
    def update_position(self, x_pixel, y_pixel, lon, lat):
        try:
            if self.rover_marker is None:
                self.rover_marker = QGraphicsEllipseItem(-5, -5, 10, 10)  # Marker size x_pixel, y_pixel
                self.rover_marker.setBrush(QBrush(QColor('blue')))
                self.customCompetitionMap.scene.addItem(self.rover_marker)
                print("Rover markerı oluşturuldu")
            else:
                scene_rect = self.customCompetitionMap.sceneRect()  # Sahne dikdörtgeni

                # Piksel koordinatlarını sahne koordinatlarına manuel olarak dönüştür
                scene_x = (x_pixel / self.customCompetitionMap.map_item.pixmap().width()) * scene_rect.width()
                scene_y = (y_pixel / self.customCompetitionMap.map_item.pixmap().height()) * scene_rect.height()
                scene_point = QPointF(scene_x, scene_y)
                self.rover_marker.setPos(scene_point) 
                
            self.ui.current_coordinates_label.setText(f"Lon: {lon:.4f}   Lat: {lat:.4f}")
        except Exception as e:
            print(f"There is an error in updating rover marker position:{e}")
        
    # "Go/Cancel" butonunun tetiklediği fonksiyon
    # İlk tıklandığında rover ilk satırdaki konuma gitmeye başlar ve bu sırada gittiği konum tabloda yeşil renk ile gözükür
    # İkinci kere tıklandığında ise bu eylem iptal edilir
    def toggle_row_highlight(self):
        if self.highlight:
            self.model.set_highlight_row(-1)
            self.ui.go_cancel_button.setText("Go")
        else:
            self.model.set_highlight_row(0)  # 0, birinci satırı belirtir
            self.ui.go_cancel_button.setText('Cancel')
        self.highlight = not self.highlight

    def sendLocation(self): 
        try:
            # x ve y verileri çekilip satır formatına getirildi
            send_lon_data = float(self.ui.send_x.text())
            send_lat_data = float(self.ui.send_y.text())

            
            new_row = [send_lon_data, send_lat_data]

            # Kutucukların boş olup olmadığı kontrol edilir ve verinin gönderildiği durumda kutucuklar temizlenir
            if send_lon_data != None and send_lat_data != None:
                self.model.addData(new_row)
                x, y = self.customCompetitionMap.gps_to_pixel(send_lat_data, send_lon_data)
                self.customCompetitionMap.add_point(x, y, str(self.model.rowCount()))   
                self.ui.send_x.clear()
                self.ui.send_y.clear()

            else:
                return None
        
        except Exception as e:
            print(f"There is an error in sending the location: {e}")

    # "Delete" tuşuna basıldığında tabloda seçilmiş satır silinir
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Delete:
            self.delete_selected_rows()

    def delete_selected_rows(self):
        try:
            selection_model = self.locationList.selectionModel()
            selected_indexes = selection_model.selectedRows()
            row = selected_indexes[0].row()
            self.model.removeRow(row)
            self.locationList.selectionModel().clearSelection()
            self.customCompetitionMap.remove_point(row) # Noktanın silinmesi
            self.update_graphics_view() # Haritanın güncellenmesi
    
        except Exception as e:
            print(f"There is an error in delete_selected_rows: {e}")

    # Tablo dışında bir yere dokunulduğunda tablodaki seçim iptal edilir
    def mousePressEvent(self, event):
        if self.locationList.rect().contains(event.pos()):
            # Tıklama tablo üzerinde ise, herhangi bir işlem yapılmaz
            self.customCompetitionMap.update_labels()
            super().mousePressEvent(event)
        else:
            # Tıklama tablo dışında ise, tablo seçimini temizleyin
            self.locationList.selectionModel().clearSelection()


    def shutdown(self):
        # İş parçacığını durdur
        self.gps_thread.stop()


def main():
    sys.exit(Main().main(sys.argv, standalone="metu_rqt_plugin.competition_map_plugin"))

if __name__ == "__main__":
    main()