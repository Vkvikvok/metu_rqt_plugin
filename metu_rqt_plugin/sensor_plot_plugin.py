from rqt_gui.main import Main
import sys
import os
from python_qt_binding.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QGridLayout, QCheckBox, QHBoxLayout
from python_qt_binding.QtCore import QTimer, Qt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import random
import rclpy
from rclpy.context import Context
from rqt_gui_py.plugin import Plugin
# Özelleştirilmiş eleman
from .sensor_subscribers_thread import SensorSubscribersThread

class SensorPlotPlugin(Plugin):
    def __init__(self, context):
        super(SensorPlotPlugin, self).__init__(context)
        self.setObjectName('SensorPlotPlugin')

        # Ana widget'ı oluştur ve ayarlarını yap
        self._widget = MyWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()



class SensorDataHandler:
    def __init__(self, filename, data_name):
        # Dosya adını ve yolunu belirleyin
        self.filename = filename
        with open(self.filename, 'a') as file:
            file.write(f"{data_name}\n")

    def write_data_to_file(self, data):
        # Dosyaya yazma işlemi
        with open(self.filename, 'a') as file:
            file.write(f"{data}\n")



class MplCanvas(FigureCanvas):
    def __init__(self, title="", parent=None, width=3, height=2, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(MplCanvas, self).__init__(self.fig)

        self.xdata = [0]
        self.ydata = [0]
        self.title = title
        self.autoscroll = True
        self.xlim_window = 30
        self.axes.set_title(title) 

    def update_data(self, new_x, new_y):
        self.xdata.append(new_x)
        self.ydata.append(new_y)
        

    def update_plot(self):
        if not self.autoscroll:
            return  # Autoscroll kapalıysa, grafiği güncellemeyi durdur

        self.axes.clear()
        self.axes.plot(self.xdata, self.ydata)
        self.axes.set_title(self.title)

        self.axes.set_xlim(left=max(0, self.xdata[-1] - self.xlim_window), right=self.xdata[-1] + 10)
        self.draw()

    def toggle_autoscroll(self, state):
        self.autoscroll = state
        if state and len(self.xdata) > 0:
            # Autoscroll açıldığında, grafiği son noktaya kaydır
            self.axes.set_xlim(left=max(0, self.xdata[-1] - self.xlim_window), right=self.xdata[-1] + 10)
            self.draw()

class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        try:
            self.layout = QGridLayout(self)

            self.canvases = []
            self.counter_list = [0,0,0,0,0,0]
            self.plot_names = ["Flamable Gas", "Methane Gas", "Carbon Monoxide", 
                            "Env. Temperature", "Env. Pressure", "Env. Humidity"]
            self.base_dir = os.path.dirname(os.path.abspath(__file__))
            self.data_files = ["flamable_gas.txt", "methane_gas.txt", "carbon_monoxide.txt", "env_temperature.txt", "env_pressure.txt", "env_humidity"]
            self.data_handlers = []
            for i in range(6):
                # Her bir sensör verisi için grafik yapısı oluşturuluyor
                title = self.plot_names[i]
                canvas = MplCanvas(title=title, width=3, height=2, dpi=100)
                self.canvases.append(canvas)

                toolbar = NavigationToolbar(canvas, self)

                # Checkbox'u toolbar'ın yanına ekliyoruz
                checkbox = QCheckBox("Autoscroll")
                checkbox.setChecked(True)
                checkbox.stateChanged.connect(lambda state, c=canvas: c.toggle_autoscroll(state == Qt.Checked))

                toolbar_layout = QHBoxLayout()
                toolbar_layout.addWidget(toolbar)
                toolbar_layout.addWidget(checkbox)

                vbox = QVBoxLayout()
                vbox.addLayout(toolbar_layout)
                vbox.addWidget(canvas)

                self.layout.addLayout(vbox, i // 2, i % 2)

                # Her bir sensör verisini depolamak için .txt dosyası oluşturuyoruz
                file_path = os.path.join(self.base_dir, "data_files", self.data_files[i])
                handler = SensorDataHandler(file_path, title)
                self.data_handlers.append(handler)

        except Exception as e:
            print(f"There is an error in initialization of sensor plot plugin:{e}")

        try:
            self.context = Context()
            self.sensor_subcribers_thread = SensorSubscribersThread(self.context)
            self.sensor_subcribers_thread.flamable_gas_signal.connect(self.send_data)
            self.sensor_subcribers_thread.methane_gas_signal.connect(self.send_data)
            self.sensor_subcribers_thread.carbon_mono_signal.connect(self.send_data)
            self.sensor_subcribers_thread.env_temp_signal.connect(self.send_data)
            self.sensor_subcribers_thread.env_pres_signal.connect(self.send_data)
            self.sensor_subcribers_thread.env_humidity_signal.connect(self.send_data)
            self.sensor_subcribers_thread.start()

        except Exception as e:
            print(f"There is an error in starting sensor subscribers thread:{e}")

        try:
            self.timer = QTimer()
            self.timer.setInterval(1000)  # 1 saniye
            self.timer.timeout.connect(self.update_plots)
            self.timer.start()
        
        except Exception as e:
            print(f"There is an error in timer:{e}")

    def send_data(self, id, data):
        self.counter_list[id] += 1
        self.canvases[id].update_data(self.counter_list[id], data)
        self.data_handlers[id].write_data_to_file(data)
        print(f"{id} indeksli grafiğe {self.counter_list[id]} zamanında {data} verisi gönderildi")


    def update_plots(self):
        for canvas in self.canvases:
            canvas.update_plot()

    def shutdown(self):
        # İş parçacığını durdur
        self.sensor_subcribers_thread.stop()


def main():
    sys.exit(Main().main(sys.argv, standalone="metu_rqt_plugin.sensor_plot_plugin"))

if __name__ == "__main__":
    main()
