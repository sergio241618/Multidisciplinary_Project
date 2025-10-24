import sys
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ====================================================================
# ===== CONFIGURATION ================================================
PORT = '/dev/ttyUSB0' # ❗ Change this to your port
BAUD = 115200
# ====================================================================

# Thread to read serial port without blocking the GUI
class SerialReader(QtCore.QThread):
    # Signal now emits three values: ref, med, control
    data_received = QtCore.pyqtSignal(float, float, float)
    reset_signal = QtCore.pyqtSignal()
    # --- NEW SIGNAL for MSE ---
    mse_received = QtCore.pyqtSignal(float) # Carries the MSE value

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None

    def run(self):
        print(f"Attempting to connect to {self.port} at {self.baud} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"Successfully connected to {self.port}.")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port: {e}")
            return

        while self.running and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # --- CHECK FOR MSE RESULT FIRST ---
                    if line.startswith("MSE_RESULT:"):
                        try:
                            mse_value = float(line.split(":")[1])
                            self.mse_received.emit(mse_value) # Emit the MSE value
                        except (IndexError, ValueError):
                            print(f"Warning: Could not parse MSE line: {line}")
                        continue # Skip processing this line as telemetry
                    # --- END OF MSE CHECK ---

                    elif "RESET" in line: # Check for RESET signal
                        self.reset_signal.emit()
                        continue # Skip processing this line as telemetry

                    # --- Process normal telemetry data ---
                    parts = line.split(',')
                    if len(parts) == 3:
                        ref = float(parts[0])
                        med = float(parts[1])
                        ctrl = float(parts[2])
                        self.data_received.emit(ref, med, ctrl)
            except (serial.SerialException, UnicodeDecodeError, ValueError):
                continue # Ignore errors and continue reading

        if self.ser.is_open:
            self.ser.close()
        print("Serial reader thread has stopped.")

    def stop(self):
        self.running = False
        self.wait()

# Main application window
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Motor Controller Diagnostic Panel')
        self.resize(1000, 800)

        # --- Layout setup (no changes) ---
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        # --- Top Plot: Velocities (no changes) ---
        self.velocity_plot = pg.PlotWidget()
        layout.addWidget(self.velocity_plot)
        self.velocity_plot.setLabel('left', 'Velocity (RPM)')
        self.velocity_plot.setLabel('bottom', 'Time (s)')
        self.velocity_plot.addLegend()
        self.velocity_plot.showGrid(x=True, y=True)
        self.velocity_plot.setXRange(0, 40, padding=0)
        self.ref_curve = self.velocity_plot.plot(pen='g', name="Reference")
        self.med_curve = self.velocity_plot.plot(pen='r', name="Measured")

        # --- Bottom Plot: Control Signal (no changes) ---
        self.control_plot = pg.PlotWidget()
        layout.addWidget(self.control_plot)
        self.control_plot.setLabel('left', 'Control Signal (u_k)')
        self.control_plot.setLabel('bottom', 'Time (s)')
        self.control_plot.showGrid(x=True, y=True)
        self.control_plot.setXRange(0, 40, padding=0)
        self.control_plot.setYRange(0, 1.1, padding=0)
        self.control_curve = self.control_plot.plot(pen='c', name="Control (u_k)")
        self.control_plot.setXLink(self.velocity_plot)

        # --- Data Buffers (no changes) ---
        self.time_data = []
        self.ref_data = []
        self.med_data = []
        self.ctrl_data = []

        # --- Start Serial Reader and Connect Signals ---
        self.serial_reader = SerialReader(PORT, BAUD)
        self.serial_reader.data_received.connect(self.update_plots)
        self.serial_reader.reset_signal.connect(self.reset_plots)
        # --- CONNECT NEW MSE SIGNAL ---
        self.serial_reader.mse_received.connect(self.display_mse) # Connect to the new slot
        self.serial_reader.start()

    # --- Slot for updating plots (no changes) ---
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

    # --- Slot for resetting plots (no changes) ---
    def reset_plots(self):
        print("\nRESET signal received. Clearing plots.")
        self.time_data.clear()
        self.ref_data.clear()
        self.med_data.clear()
        self.ctrl_data.clear()
        self.ref_curve.setData([], [])
        self.med_curve.setData([], [])
        self.control_curve.setData([], [])

    # --- NEW SLOT for displaying MSE ---
    def display_mse(self, mse_value):
        """Prints the received MSE value to the console."""
        print("\n===================================")
        # Determine which controller was active based on ESP32's define
        # We can't know for sure here, so we print a generic message
        print(f"Run Complete. Mean Squared Error (MSE): {mse_value:.4f}")
        print("===================================\n")

    # --- Close event (no changes) ---
    def closeEvent(self, event):
        print("Window closed. Stopping serial reader thread...")
        self.serial_reader.stop()
        event.accept()

# --- Main entry point (no changes) ---
if __name__ == '__main__':
    TS_MS = 10
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())