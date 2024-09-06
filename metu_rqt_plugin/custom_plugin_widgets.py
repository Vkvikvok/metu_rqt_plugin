# Bu dosyada bazı arayüz elemanlarına ek özellikler tanımlanmaktadır ve sonrasında değiştirilmiş
# elemanlar ui dosyasındaki asıl eleman sınfıyla değiştirilmiştir.

from python_qt_binding.QtWidgets import QGraphicsView, QGraphicsPixmapItem, QTableView, QGraphicsEllipseItem, QGraphicsTextItem, QListView
from python_qt_binding.QtCore import Qt, QRectF, QAbstractTableModel, QModelIndex, QVariant, QMimeData, QDataStream, QByteArray, pyqtSignal
from python_qt_binding.QtGui import QPainter, QPixmap, QDrag, QBrush, QColor, QStandardItemModel, QStandardItem, QPen

# İçerisinde harita öğelerini taşıyacak özelleştirilmiş eleman
class CustomGraphicsView(QGraphicsView):
    
    def __init__(self, scene, parent=None, box_x=None, box_y=None): 
        super(CustomGraphicsView, self).__init__(scene, parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.NoDrag)
        self.scene = scene
        self.setScene(self.scene)
        self._isPanning = False
        self.box_x = box_x
        self.box_y = box_y
        self.points = []

        # Harita bilgileri(Güncellenmeli)
        # Burada enlemleri ters tanımlamamın sebebi piksellerin de koordinat sisteminin gps enlemine göre ters olması
        # Enlemlerin negatifleri kullanılacak
        self.max_lat = -50.066332 # Haritanın sol üst köşesinin derece cinsinden enlemi(Kuzey negatif, Güney pozitif)
        self.min_lat = -50.065984 # Haritanın sağ alt köşesinin derece cinsinden enlemi
        self.min_lon = 19.913229 # Haritanın sol üst köşesinin derece cinsinden boylamı(Doğu pozitif, Batı negatif)
        self.max_lon = 19.913646 # Haritanın sağ alt köşesinin derece cinsinden boylamı


    #Panning 
    def mousePressEvent(self, event):
        try:
            if event.button() == Qt.MiddleButton:
                self._isPanning = True
                self.setCursor(Qt.ClosedHandCursor)
                self._lastPos = event.pos()
            if event.button() == Qt.LeftButton:
                scene_pos = self.mapToScene(event.pos())
                self.update_coordinates(scene_pos.x(), scene_pos.y()) 
            super(CustomGraphicsView, self).mousePressEvent(event)

        except Exception as e:
            print(f"Error in mousePressEvent: {e}")

    # Haritadan bir nokta seçildiğinde koordinatlar gerekli kutucuklara otomatik olarak girilir
    def update_coordinates(self, x, y):
        try:
            lon = x*(self.max_lon-self.min_lon)/self.map_item.pixmap().width() + self.min_lon
            lat = -y*(self.max_lat-self.min_lat)/self.map_item.pixmap().height() + self.max_lat
            self.box_x.setText(f"{lon:.6f}")
            self.box_y.setText(f"{-lat:.6f}")
        
        except Exception as e:
            print(f"There is an error in update_coordinates:{e}")

    def mouseMoveEvent(self, event):
        try:
            if self._isPanning:
                delta = event.pos() - self._lastPos
                self.translateView(delta)
                self._lastPos = event.pos()
            super(CustomGraphicsView, self).mouseMoveEvent(event)

        except Exception as e:
            print(f"Error in mouseMoveEvent: {e}")

    def mouseReleaseEvent(self, event):
        try:
            if event.button() == Qt.MiddleButton:
                self._isPanning = False
                self.setCursor(Qt.ArrowCursor)
            super(CustomGraphicsView, self).mouseReleaseEvent(event)

        except Exception as e:
            print(f"Error in mouseReleaseEvent: {e}")

    def translateView(self, delta):
        try:
            self.setTransformationAnchor(QGraphicsView.NoAnchor)
            self.setResizeAnchor(QGraphicsView.NoAnchor)
            self.translate(delta.x(), delta.y())

        except Exception as e:
            print(f"Error in translateView: {e}")

    #Zooming
    def wheelEvent(self, event):
        try:
            factor = 1.2
            if event.angleDelta().y() < 0:
                factor = 1.0 / factor
            self.scale(factor, factor)

        except Exception as e:
            print(f"Error in wheelEvent: {e}")

    #Harita görselini yükleme
    def load_image(self, image_path):
        try:
            self.pixmap = QPixmap(image_path)
            scaled_pixmap = self.pixmap.scaled(self.viewport().size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.scene.clear()
            self.map_item = QGraphicsPixmapItem(scaled_pixmap)
            self.scene.addItem(self.map_item)
            self.scene.setSceneRect(self.map_item.boundingRect())

        except Exception as e:
            print(f"Error in load_image: {e}")

    # Haritada gidilecek konumlara nokta ekleme
    def add_point(self, x, y, label):
        
        point_radius = 5
        ellipse = QGraphicsEllipseItem(QRectF(x - point_radius, y - point_radius, point_radius * 2, point_radius * 2))
        ellipse.setBrush(Qt.red)
        text = QGraphicsTextItem(label)
        text.setPos(x + 5, y - 5)
        self.scene.addItem(ellipse)
        self.scene.addItem(text)
        self.points.append((ellipse, text))

    # Haritadaki noktalardan birisini sil
    def remove_point(self, index):
        if 0 <= index < len(self.points):
            ellipse, text = self.points.pop(index)
            self.scene.removeItem(ellipse)
            self.scene.removeItem(text)

    # Tablo güncellendiğinde haritadaki noktaları yeniden numaralandır
    def update_points(self, data):
        for index, (item, label) in enumerate(self.points):
            x, y = data[index]
            item.setRect(x - 5, y - 5, 10, 10)
            label.setPos(x + 10, y)
            label.setPlainText(str(index + 1))

    def gps_to_pixel(self, latitude, longitude):
        x = (longitude - self.min_lon) * self.map_item.pixmap().width() / (self.max_lon - self.min_lon)
        # Aşağıda enlemler negatif alındığından dolayı burada da "min_lat" ile "max_lat" değerlerinin yerlerini değiştirerek kullandım
        y = (latitude + self.max_lat) * self.map_item.pixmap().height() / (self.max_lat - self.min_lat)         
        return x, y

# İçerisinde roverın gitmesi gereken konumları içeren tablonun modeli
class MyTableModel(QAbstractTableModel):
    def __init__(self, max_lat, min_lat, max_lon, min_lon, map_width, map_height, data=None, parent=None):
        super(MyTableModel, self).__init__(parent)
        self._data = data if data is not None else []
        self.pixel_data = []
        self.headers = ["Lon", "Lat"]
        self.highlight_row = -1 # "Go" komudu verildiğinde doğru satırın etkilenmesini sağlayacak değişken
        self.max_lat = max_lat
        self.min_lat = min_lat
        self.max_lon = max_lon
        self.min_lon = min_lon
        self.map_width = map_width
        self.map_height = map_height

    def rowCount(self, parent=QModelIndex()):
        return len(self._data)
    
    def columnCount(self, parent=QModelIndex()):
        return len(self.headers)
    
    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return QVariant()
        
        if role == Qt.DisplayRole:
            value = self._data[index.row()][index.column()]
            return QVariant(value)
        
        if role == Qt.BackgroundRole:
            # Satırın arka plan rengini belirler
            if index.row() == self.highlight_row:
                return QBrush(QColor('#20bd0f'))
        return QVariant()
    
    # İlk satırın renk değişiminde rol oynar
    def set_highlight_row(self, row):
        self.highlight_row = row
        self.dataChanged.emit(QModelIndex(), QModelIndex())
    
    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                if 0 <= section < len(self.headers):
                    return QVariant(self.headers[section])
            
            if orientation == Qt.Vertical:
                return str(section + 1)
        return QVariant()
    
    # Verilerin değiştiği durumlarda tablonun yenilenmesini sağlayan kısım
    def setData(self, index, value, role):
        if role == Qt.EditRole:
            self._data[index.row()][index.column()] = value
            self.dataChanged.emit(index, index, [Qt.DisplayRole])
            return True
        return False
    
    def swapRows(self, row1, row2):
        self.beginMoveRows(QModelIndex(), row1, row1, QModelIndex(), row2 + 1 if row1 < row2 else row2)
        self._data[row1], self._data[row2] = self._data[row2], self._data[row1]
        self.pixel_data[row1], self.pixel_data[row2] = self.pixel_data[row2], self.pixel_data[row1]
        self.endMoveRows()
        self.dataChanged.emit(self.index(0, 0), self.index(self.rowCount(None) - 1, self.columnCount(None) - 1), [Qt.DisplayRole])

    def clearSelection(self):
        self.selected_row = None
        self.dataChanged.emit(self.index(0, 0), self.index(self.rowCount(None) - 1, self.columnCount(None) - 1), [Qt.BackgroundRole])

    def addData(self, new_row):
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._data.append(new_row)
        lon, lat = new_row
        self.pixel_data.append(self.gps_to_pixel(lat, lon))
        self.endInsertRows()

    def removeRow(self, row):
        self.beginRemoveRows(QModelIndex(), row, row)
        self._data.pop(row)
        self.pixel_data.pop(row)
        self.endRemoveRows()

    def gps_to_pixel(self, latitude, longitude):
        x = (longitude - self.min_lon) * self.map_width / (self.max_lon - self.min_lon)
        # Aşağıda enlemler negatif alındığından dolayı burada da "min_lat" ile "max_lat" değerlerinin yerlerini değiştirerek kullandım
        y = (latitude + self.max_lat) * self.map_height / (self.max_lat - self.min_lat)         
        return [x,y]

# Gidilecek konumların listesini içeren tablonun ekstra özellikleri tanımlandı
class CustomTableView(QTableView):
    def __init__(self, parent=None):
        super(CustomTableView, self).__init__(parent)
        self.double_clicked_row = None

    # Bir satıra çift tıklama durumunda başka bir satırla yer değiştirme için tetiklenir
    def mouseDoubleClickEvent(self, event):
        index = self.indexAt(event.pos())
        if index.isValid():
            self.double_clicked_row = index.row()
            self.model().selected_row = self.double_clicked_row
            self.model().dataChanged.emit(self.model().index(0, 0), self.model().index(self.model().rowCount(None) - 1, self.model().columnCount(None) - 1), [Qt.BackgroundRole])
        super(CustomTableView, self).mouseDoubleClickEvent(event)

    # Çift tıklamanın ardından başka bir hücreye sol click ile tıklandığında satırlar yer değiştirir,
    # sağ click durumunda ise seçim iptal edilir 
    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton and self.double_clicked_row is not None:
            self.model().clearSelection()
            self.double_clicked_row = None
        elif event.button() == Qt.LeftButton and self.double_clicked_row is not None:
            index = self.indexAt(event.pos())
            if index.isValid() and index.row() != self.double_clicked_row:
                self.model().swapRows(self.double_clicked_row, index.row())
                self.model().clearSelection()
                self.double_clicked_row = None
        super(CustomTableView, self).mousePressEvent(event)


class CustomListView(QListView):
    def __init__(self, model):
        super(CustomListView, self).__init__()
        self.setModel(model)
        self.setSelectionMode(QListView.NoSelection)  # Seçimi tamamen kapatmak için
        self.setEditTriggers(QListView.NoEditTriggers)  # Düzenlemeyi kapatmak için
    
    def mousePressEvent(self, event):
        # Eğer sağ tıklama ise
        if event.button() == Qt.RightButton:
            index = self.indexAt(event.pos())
            if index.isValid():
                self.model().removeRow(index.row())
        
        # Sol tıklamayı yok saymak için mousePressEvent'i çağırmıyoruz
        # Sağ tıklama dışındaki durumlarda varsayılan davranışı kullanabiliriz
        else:
            super(CustomListView, self).mousePressEvent(event)