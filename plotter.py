import sys
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ====================================================================
# ===== CONFIGURACIÓN ================================================
PORT = '/dev/ttyUSB0' # ❗ Cambia esto a tu puerto
BAUD = 115200
# ====================================================================

# Hilo para leer el puerto serie sin bloquear la interfaz
class SerialReader(QtCore.QThread):
    # Señal ahora emite tres valores: ref, med, control
    data_received = QtCore.pyqtSignal(float, float, float) 
    reset_signal = QtCore.pyqtSignal()

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"Conectado a {self.port} a {self.baud} baudios.")
        except serial.SerialException as e:
            print(f"Error al abrir el puerto serial: {e}")
            return

        while self.running and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    if "RESET" in line:
                        self.reset_signal.emit()
                        continue
                    
                    parts = line.split(',')
                    # AHORA ESPERAMOS TRES PARTES
                    if len(parts) == 3:
                        ref = float(parts[0])
                        med = float(parts[1])
                        ctrl = float(parts[2])
                        self.data_received.emit(ref, med, ctrl) # Emitimos los tres valores
            except (serial.SerialException, UnicodeDecodeError, ValueError):
                continue
        
        if self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.running = False
        self.wait()

# Ventana principal de la aplicación
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Panel de Diagnóstico del Controlador')
        self.resize(1000, 800)

        # --- Layout para dos gráficas, una encima de la otra ---
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        # --- Gráfica Superior: Velocidades ---
        self.velocity_plot = pg.PlotWidget()
        layout.addWidget(self.velocity_plot)
        self.velocity_plot.setLabel('left', 'Velocidad (RPM)')
        self.velocity_plot.setLabel('bottom', 'Tiempo (s)')
        self.velocity_plot.addLegend()
        self.velocity_plot.showGrid(x=True, y=True)
        self.velocity_plot.setXRange(0, 40, padding=0) # Asumiendo 40s de trayectoria
        
        self.ref_curve = self.velocity_plot.plot(pen='g', name="Referencia")
        self.med_curve = self.velocity_plot.plot(pen='r', name="Medida")

        # --- Gráfica Inferior: Señal de Control ---
        self.control_plot = pg.PlotWidget()
        layout.addWidget(self.control_plot)
        self.control_plot.setLabel('left', 'Señal de Control (u_k)')
        self.control_plot.setLabel('bottom', 'Tiempo (s)')
        self.control_plot.showGrid(x=True, y=True)
        self.control_plot.setXRange(0, 40, padding=0)
        self.control_plot.setYRange(0, 1.1, padding=0) # u_k está entre 0 y 1

        self.control_curve = self.control_plot.plot(pen='c', name="Control (u_k)")
        
        # Sincronizar los ejes X de ambas gráficas
        self.control_plot.setXLink(self.velocity_plot)

        # --- Buffers de datos (ahora 3) ---
        self.time_data = []
        self.ref_data = []
        self.med_data = []
        self.ctrl_data = []
        
        # Iniciar el lector del puerto serie
        self.serial_reader = SerialReader(PORT, BAUD)
        self.serial_reader.data_received.connect(self.update_plots)
        self.serial_reader.reset_signal.connect(self.reset_plots)
        self.serial_reader.start()

    def update_plots(self, ref, med, ctrl):
        current_time = 0
        if self.time_data:
            current_time = self.time_data[-1] + (TS_MS / 1000.0)
        
        self.time_data.append(current_time)
        self.ref_data.append(ref)
        self.med_data.append(med)
        self.ctrl_data.append(ctrl)
        
        self.ref_curve.setData(self.time_data, self.ref_data)
        self.med_curve.setData(self.time_data, self.med_data)
        self.control_curve.setData(self.time_data, self.ctrl_data)

    def reset_plots(self):
        print("Señal de RESET recibida. Reiniciando gráficas.")
        self.time_data.clear()
        self.ref_data.clear()
        self.med_data.clear()
        self.ctrl_data.clear()
        
        self.ref_curve.setData([], [])
        self.med_curve.setData([], [])
        self.control_curve.setData([], [])

    def closeEvent(self, event):
        self.serial_reader.stop()
        event.accept()

if __name__ == '__main__':
    TS_MS = 10 
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())