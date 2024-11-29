# import sys
# import usb.core
# import usb.util
# from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidget, QPushButton, QVBoxLayout, QWidget, QMessageBox

# class USBDeviceManager(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         self.setWindowTitle("USB Device Manager")
#         self.setGeometry(100, 100, 400, 300)

#         self.central_widget = QWidget(self)
#         self.setCentralWidget(self.central_widget)

#         self.layout = QVBoxLayout(self.central_widget)

#         self.device_list = QListWidget(self.central_widget)
#         self.layout.addWidget(self.device_list)

#         self.scan_button = QPushButton("Scan USB Devices", self)
#         self.scan_button.clicked.connect(self.scan_devices)
#         self.layout.addWidget(self.scan_button)

#     def scan_devices(self):
#         self.device_list.clear()  # 이전 목록 지우기
#         devices = list(usb.core.find(find_all=True))

#         if not devices:
#             QMessageBox.information(self, "Information", "No USB devices found.")
#             return

#         for device in devices:
#             try:
#                 device_name = usb.util.get_string(device, device.iProduct) if device.iProduct else "N/A"
#                 manufacturer_name = usb.util.get_string(device, device.iManufacturer) if device.iManufacturer else "N/A"
#                 display_name = f"Vendor: {hex(device.idVendor)}, Product: {hex(device.idProduct)}, Name: {device_name}, Manufacturer: {manufacturer_name}"
#                 self.device_list.addItem(display_name)
#             except Exception as e:
#                 print(f"Error retrieving information for device idVendor={hex(device.idVendor)}, idProduct={hex(device.idProduct)}: {e}")

#         self.device_list.itemClicked.connect(self.connect_device)

#     def connect_device(self, item):
#         selected_text = item.text()
#         QMessageBox.information(self, "Connect Device", f"Connecting to: {selected_text}")

#         # 여기에서 실제 연결 로직을 추가할 수 있습니다.
#         # 예를 들어, USB 장치와의 실제 연결을 설정하는 코드를 추가합니다.

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = USBDeviceManager()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# import usb.core
# import usb.util
# from PyQt5.QtWidgets import QApplication, QMainWindow, QComboBox, QPushButton, QVBoxLayout, QWidget, QLabel, QMessageBox

# class USBDeviceManager(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         self.setWindowTitle("USB Device Manager")
#         self.setGeometry(100, 100, 400, 200)

#         self.central_widget = QWidget(self)
#         self.setCentralWidget(self.central_widget)

#         self.layout = QVBoxLayout(self.central_widget)

#         self.selected_device_label = QLabel("Selected Device: None", self)
#         self.layout.addWidget(self.selected_device_label)

#         self.device_combo_box = QComboBox(self)
#         self.layout.addWidget(self.device_combo_box)

#         self.scan_button = QPushButton("↓ Scan USB Devices", self)
#         self.scan_button.clicked.connect(self.scan_devices)
#         self.layout.addWidget(self.scan_button)

#         # 선택된 장치가 변경될 때 호출되는 슬롯 연결
#         self.device_combo_box.currentIndexChanged.connect(self.device_selected)

#     def scan_devices(self):
#         self.device_combo_box.clear()  # 이전 목록 지우기
#         devices = list(usb.core.find(find_all=True))

#         if not devices:
#             QMessageBox.information(self, "Information", "No USB devices found.")
#             return

#         for device in devices:
#             try:
#                 device_name = usb.util.get_string(device, device.iProduct) if device.iProduct else "N/A"
#                 manufacturer_name = usb.util.get_string(device, device.iManufacturer) if device.iManufacturer else "N/A"
#                 display_name = f"Vendor: {hex(device.idVendor)}, Product: {hex(device.idProduct)}, Name: {device_name}, Manufacturer: {manufacturer_name}"
#                 self.device_combo_box.addItem(display_name)
#             except Exception as e:
#                 print(f"Error retrieving information for device idVendor={hex(device.idVendor)}, idProduct={hex(device.idProduct)}: {e}")

#     def device_selected(self):
#         # 선택된 장치 이름을 가져와서 QLabel에 표시
#         current_device = self.device_combo_box.currentText()
#         self.selected_device_label.setText(f"Selected Device: {current_device}")

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = USBDeviceManager()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# import time
# import random
# from PyQt5.QtWidgets import QApplication, QMainWindow, QTableView, QVBoxLayout, QWidget, QLabel
# from PyQt5.QtCore import QThread, pyqtSignal, Qt
# from PyQt5.QtGui import QStandardItemModel, QStandardItem

# class DataGenerator(QThread):
#     data_updated = pyqtSignal(list)

#     def run(self):
#         while True:
#             # 예시 데이터 생성 (여기서 실제 데이터 수집 로직을 대체)
#             data = [random.randint(0, 100) for _ in range(5)]  # 5개의 임의의 데이터 생성
#             self.data_updated.emit(data)
#             time.sleep(0.0001)  # 100μs 대기

# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         self.setWindowTitle("Real-Time Data Update Example")
#         self.setGeometry(100, 100, 400, 300)

#         self.model = QStandardItemModel(0, 5, self)  # 5개의 열
#         self.model.setHorizontalHeaderLabels(["Column 1", "Column 2", "Column 3", "Column 4", "Column 5"])  # 열 이름 설정

#         self.setCentralWidget(QWidget(self))
#         self.layout = QVBoxLayout(self.centralWidget())
#         self.table_view = QTableView(self)
#         self.table_view.setModel(self.model)
#         self.layout.addWidget(self.table_view)

#         self.data_generator = DataGenerator()
#         self.data_generator.data_updated.connect(self.update_table)
#         self.data_generator.start()

#     def update_table(self, data):
#         self.model.setRowCount(0)  # 이전 데이터 지우기
#         self.model.appendRow([QStandardItem(str(value)) for value in data])  # 새로운 데이터 추가

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = MainWindow()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# from PyQt5.QtWidgets import QApplication, QTableView, QVBoxLayout, QWidget
# from PyQt5.QtGui import QStandardItemModel, QStandardItem

# class TableViewExample(QWidget):
#     def __init__(self):
#         super().__init__()

#         # QTableView 및 모델 초기화
#         self.table_view = QTableView()
#         self.model = QStandardItemModel(5, 3)  # 5행, 3열
#         self.table_view.setModel(self.model)

#         # 데이터 추가
#         for row in range(5):
#             for column in range(3):
#                 item = QStandardItem(f"Item {row}, {column}")
#                 self.model.setItem(row, column, item)

#         # 수평 헤더 크기 설정 (각 열별로 다르게 설정)
#         self.table_view.setColumnWidth(0, 100)  # 첫 번째 열 너비 100픽셀
#         self.table_view.setColumnWidth(1, 150)  # 두 번째 열 너비 150픽셀
#         self.table_view.setColumnWidth(2, 200)  # 세 번째 열 너비 200픽셀

#         # 수직 헤더 크기 설정 (각 행별로 다르게 설정)
#         self.table_view.setRowHeight(0, 30)  # 첫 번째 행 높이 30픽셀
#         self.table_view.setRowHeight(1, 40)  # 두 번째 행 높이 40픽셀
#         self.table_view.setRowHeight(2, 50)  # 세 번째 행 높이 50픽셀
#         self.table_view.setRowHeight(3, 60)  # 네 번째 행 높이 60픽셀
#         self.table_view.setRowHeight(4, 70)  # 다섯 번째 행 높이 70픽셀

#         # 레이아웃 설정
#         layout = QVBoxLayout()
#         layout.addWidget(self.table_view)
#         self.setLayout(layout)

#         self.setWindowTitle("QTableView Example")
#         self.resize(400, 300)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = TableViewExample()
#     window.show()
#     sys.exit(app.exec_())

# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSpinBox, QLabel

# class SpinBoxExample(QWidget):
#     def __init__(self):
#         super().__init__()

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # QLabel 생성
#         self.label = QLabel("현재 값: 0")
#         layout.addWidget(self.label)

#         # QSpinBox 생성
#         self.spin_box = QSpinBox()
#         self.spin_box.setRange(0, 100)  # 범위 설정 (최소 0, 최대 100)
#         self.spin_box.setValue(0)  # 초기값 설정
#         layout.addWidget(self.spin_box)

#         # 값 변경 시 호출될 메서드 연결
#         self.spin_box.valueChanged.connect(self.update_label)

#         self.setLayout(layout)
#         self.setWindowTitle("QSpinBox Example")
#         self.resize(200, 100)

#     def update_label(self, value):
#         # QLabel 업데이트
#         self.label.setText(f"현재 값: {value}")

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = SpinBoxExample()
#     window.show()
#     sys.exit(app.exec_())

# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QListView, QMessageBox
# from PyQt5.QtCore import QStringListModel, Qt

# class MainWindow(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Motor List Example")
#         self.setGeometry(100, 100, 300, 300)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # QListView 생성
#         self.detected_motor_list = QListView()
#         self.model = QStringListModel()  # 문자열 리스트 모델
#         self.detected_motor_list.setModel(self.model)  # QListView에 모델 설정
#         layout.addWidget(self.detected_motor_list)

#         # 버튼 생성
#         self.add_button = QPushButton("Add Motor")
#         self.add_button.clicked.connect(self.add_motor)  # 버튼 클릭 시 add_motor 호출
#         layout.addWidget(self.add_button)

#         # 클리어 버튼 생성
#         self.clear_button = QPushButton("Clear List")
#         self.clear_button.clicked.connect(self.clear_motor_list)  # 버튼 클릭 시 clear_motor_list 호출
#         layout.addWidget(self.clear_button)

#         # QListView의 항목 클릭 시 호출될 메서드 연결
#         self.detected_motor_list.clicked.connect(self.on_item_clicked)

#         self.setLayout(layout)

#         # 초기 데이터 설정
#         self.motor_list = []  # 모터 데이터를 저장할 리스트

#     def add_motor(self):
#         # 새로운 모터 데이터 추가
#         new_motor = f"Motor {len(self.motor_list) + 1}"
#         self.motor_list.append(new_motor)
#         self.model.setStringList(self.motor_list)  # 모델에 데이터 설정

#     def clear_motor_list(self):
#         # 리스트 클리어
#         self.motor_list.clear()  # 리스트 비우기
#         self.model.setStringList(self.motor_list)  # 모델 업데이트

#     def on_item_clicked(self, index):
#         # 클릭한 항목의 데이터 가져오기
#         motor_name = self.model.data(index, Qt.DisplayRole)  # role 인자로 Qt.DisplayRole 추가
#         self.show_message(motor_name)

#     def show_message(self, motor_name):
#         # 클릭한 모터 이름을 메시지 박스로 표시
#         QMessageBox.information(self, "Motor Selected", f"You clicked: {motor_name}")

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     main_window = MainWindow()
#     main_window.show()
#     sys.exit(app.exec_())

# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QDialog

# class NewWindow(QDialog):  # QDialog를 상속받음
#     def __init__(self, window_number):
#         super().__init__()
#         self.setWindowTitle(f"New Window {window_number}")
#         self.setGeometry(150, 150, 200, 100)
        
#         layout = QVBoxLayout()
#         label = QLabel(f"This is window {window_number}")
#         layout.addWidget(label)
#         self.setLayout(layout)

# class MainWindow(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Main Window")
#         self.setGeometry(100, 100, 300, 200)

#         layout = QVBoxLayout()
#         self.button = QPushButton("Open New Window")
#         self.button.clicked.connect(self.open_new_window)
#         layout.addWidget(self.button)

#         self.setLayout(layout)

#         self.window_count = 0

#     def open_new_window(self):
#         self.window_count += 1
#         new_window = NewWindow(self.window_count)
#         new_window.exec_()  # 모달로 실행

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     main_window = MainWindow()
#     main_window.show()
#     sys.exit(app.exec_())


# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
# from PyQt5.QtCore import QTimer

# class TimerExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.data = 0
#         self.setWindowTitle("QTimer Example")
#         self.setGeometry(100, 100, 300, 200)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # QLabel 생성
#         self.label = QLabel("Timer not started")
#         layout.addWidget(self.label)

#         # QTimer 생성
#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_label)  # 타이머가 만료되면 호출될 메서드 연결

#         # 시작 버튼 생성
#         self.start_button = QPushButton("Start Timer")
#         self.start_button.clicked.connect(self.start_timer)  # 버튼 클릭 시 타이머 시작
#         layout.addWidget(self.start_button)

#         # 정지 버튼 생성
#         self.stop_button = QPushButton("Stop Timer")
#         self.stop_button.clicked.connect(self.stop_timer)  # 버튼 클릭 시 타이머 정지
#         layout.addWidget(self.stop_button)

#         self.setLayout(layout)

#     def start_timer(self):
#         self.label.setText("Timer started")
#         self.timer.start(int(1000/30))  # 1000ms (1초) 간격으로 타이머 시작

#     def stop_timer(self):
#         self.timer.stop()  # 타이머 정지
#         self.label.setText("Timer stopped")

#     def update_label(self):
#         te = "Timer ticked! " + str(self.data)
#         self.label.setText(te)  # 타이머가 만료될 때 호출
#         self.data += 1
        
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = TimerExample()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
# from PyQt5.QtCore import QThread, pyqtSignal
# import time

# class Worker(QThread):
#     # 신호 정의: 작업이 완료되면 GUI에 결과를 전달하기 위해 사용
#     update_signal = pyqtSignal(int)

#     def run(self):
#         count = 0
#         while count < 10:
#             time.sleep(1)  # 1초 대기
#             count += 1
#             self.update_signal.emit(count)  # 신호를 통해 GUI에 업데이트

# class MainWindow(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("QThread Example")
#         self.setGeometry(100, 100, 300, 200)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # QLabel 생성
#         self.label = QLabel("Count: 0")
#         layout.addWidget(self.label)

#         # 시작 버튼 생성
#         self.start_button = QPushButton("Start Counting")
#         self.start_button.clicked.connect(self.start_counting)
#         layout.addWidget(self.start_button)

#         self.setLayout(layout)

#         # Worker 스레드 인스턴스 생성
#         self.worker = Worker()
#         self.worker.update_signal.connect(self.update_label)  # 신호와 슬롯 연결

#     def start_counting(self):
#         self.label.setText("Counting...")
#         self.worker.start()  # 스레드 시작

#     def update_label(self, count):
#         self.label.setText(f"Count: {count}")  # 레이블 업데이트

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = MainWindow()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# import random
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
# from PyQt5.QtGui import QPainter, QPen
# from PyQt5.QtCore import Qt, QPoint, QTimer

# class GraphWidget(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Real-time Graph Example")
#         self.setGeometry(100, 100, 400, 300)
#         self.data = [0] * 10  # 초기 데이터 (10개의 0)
        
#         # QTimer 설정
#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_graph)  # 타이머가 만료될 때 호출할 메서드 연결
#         self.timer.start(10)  # 1초마다 데이터 업데이트

#     def paintEvent(self, event):
#         painter = QPainter(self)
#         self.draw_graph(painter)

#     def draw_graph(self, painter):
#         # 그래프의 배경 색상
#         painter.fillRect(self.rect(), Qt.white)

#         # 선 색상 및 두께 설정
#         pen = QPen(Qt.blue, 2)
#         painter.setPen(pen)

#         # 데이터 포인트 계산
#         width = self.width()
#         height = self.height()
#         margin = 20

#         # 그래프의 시작점
#         prev_point = QPoint(margin, height - margin - int(self.data[0]))

#         # 데이터 포인트를 연결하여 선 그리기
#         for i in range(1, len(self.data)):
#             x = margin + (width - 2 * margin) * i / (len(self.data) - 1)
#             y = height - margin - int(self.data[i])  # int로 변환
#             curr_point = QPoint(int(x), int(y))  # QPoint에 int로 변환된 값 사용
#             painter.drawLine(prev_point, curr_point)  # 이전 점과 현재 점을 연결
#             prev_point = curr_point

#     def update_graph(self):
#         # 새로운 랜덤 데이터 생성
#         self.data.append(random.randint(0, 100))  # 0-100 범위의 랜덤 값 추가
#         if len(self.data) > 100:  # 데이터 길이가 10을 초과하면 가장 오래된 데이터 제거
#             self.data.pop(0)
        
#         self.update()  # 위젯을 업데이트하여 다시 그리기

# class MainWindow(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Real-time Graph Example")
#         self.setGeometry(100, 100, 400, 300)

#         layout = QVBoxLayout()
        
#         self.graph_widget = GraphWidget()
#         layout.addWidget(self.graph_widget)

#         self.setLayout(layout)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     main_window = MainWindow()
#     main_window.show()
#     sys.exit(app.exec_())


# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QDial, QDoubleSpinBox, QLabel

# class DialSpinBoxExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Dial and Double Spin Box Example")
#         self.setGeometry(100, 100, 300, 200)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # QDial 생성
#         self.dial = QDial()
#         self.dial.setRange(0, 100)  # 다이얼의 범위 설정 (0에서 100)
#         self.dial.setValue(50)  # 초기값 설정
#         layout.addWidget(self.dial)

#         # QDoubleSpinBox 생성
#         self.spin_box = QDoubleSpinBox()
#         self.spin_box.setRange(0, 100)  # 스핀 박스의 범위 설정 (0에서 100)
#         self.spin_box.setValue(50.0)  # 초기값 설정
#         layout.addWidget(self.spin_box)

#         # 다이얼과 스핀 박스의 값 변경 시 호출될 메서드 연결
#         self.dial.valueChanged.connect(self.update_spin_box)
#         self.spin_box.valueChanged.connect(self.update_dial)

#         self.setLayout(layout)

#     def update_spin_box(self, value):
#         # 다이얼의 값에 맞춰 스핀 박스의 값을 업데이트
#         self.spin_box.setValue(float(value))

#     def update_dial(self, value):
#         # 스핀 박스의 값에 맞춰 다이얼의 값을 업데이트
#         self.dial.setValue(int(value))

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = DialSpinBoxExample()
#     window.show()
#     sys.exit(app.exec_())

# import can
# import usb
# from can import Listener

# # 수신된 메시지를 처리하는 콜백 함수
# def on_message_received(msg):
#     print("Received message:", msg)

# # 사용자 정의 Listener 클래스
# class MyListener(Listener):
#     def on_message_received(self, msg):
#         super().on_message_received(msg)  # 기본 동작 호출
#         on_message_received(msg)  # 사용자 정의 콜백 함수 호출

# def main():
#     dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
#     # CAN Bus 인터페이스 설정
#     bus = can.interface.Bus(interface="gs_usb", channel="canable gs_usb", index=0, bitrate=1000000)

#     # Listener 인스턴스 생성
#     listener = MyListener()
#     notifier = can.Notifier(bus, [listener])  # Notifier를 사용하여 메시지 수신 대기

#     try:
#         # 메인 스레드에서 다른 작업 수행 가능
#         print("Listening for messages...")
#         while True:
#             pass  # 다른 작업을 수행하거나 대기
#     except KeyboardInterrupt:
#         print("Exiting...")
#     finally:
#         notifier.stop()  # Notifier 종료
#         bus.shutdown()  # CAN Bus 종료

# if __name__ == "__main__":
#     main()

# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSizePolicy, QSpacerItem

# class SpacerExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Spacer Example")
#         self.setGeometry(100, 100, 300, 200)

#         # 수직 레이아웃 생성
#         vbox = QVBoxLayout()

#         # 수평 레이아웃 생성
#         hbox = QHBoxLayout()

#         # 버튼 생성
#         button1 = QPushButton("Button 1")
#         button2 = QPushButton("Button 2")

#         # 수평 스페이서 추가
#         h_spacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
#         hbox.addWidget(button1)
#         hbox.addItem(h_spacer)  # 버튼 사이에 수평 스페이서 추가
#         hbox.addWidget(button2)

#         # 수직 스페이서 추가
#         v_spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
#         vbox.addItem(v_spacer)  # 수직 스페이서 추가
#         vbox.addLayout(hbox)

#         self.setLayout(vbox)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = SpacerExample()
#     window.show()
#     sys.exit(app.exec_())

# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel

# class ResizeExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Resize Example")
#         self.setGeometry(100, 100, 400, 300)

#         # 수직 레이아웃 생성
#         vbox = QVBoxLayout()

#         # 수평 레이아웃 생성
#         hbox = QHBoxLayout()

#         # 버튼과 레이블 생성
#         button1 = QPushButton("Button 1")
#         button2 = QPushButton("Button 2")
#         label = QLabel("Resize the window!")

#         # 레이아웃에 위젯 추가
#         hbox.addWidget(button1)
#         hbox.addWidget(button2)

#         vbox.addLayout(hbox)
#         vbox.addWidget(label)

#         # 메인 레이아웃 설정
#         self.setLayout(vbox)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = ResizeExample()
#     window.show()
#     sys.exit(app.exec_())


# print(hex(int("A4", 16)))
# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLineEdit, QPushButton, QLabel

# class LineEditExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("QLineEdit Example")
#         self.setGeometry(100, 100, 300, 150)

#         # 레이아웃 생성
#         layout = QVBoxLayout()

#         # QLineEdit 생성
#         self.line_edit = QLineEdit()
#         self.line_edit.setPlaceholderText("Enter text here...")  # 플레이스홀더 텍스트 설정

#         # QPushButton 생성
#         self.button = QPushButton("Show Text")
#         self.button.clicked.connect(self.show_text)  # 버튼 클릭 시 show_text 메서드 호출

#         # QLabel 생성
#         self.label = QLabel("Your text will appear here.")

#         # 레이아웃에 위젯 추가
#         layout.addWidget(self.line_edit)
#         layout.addWidget(self.button)
#         layout.addWidget(self.label)

#         # 메인 레이아웃 설정
#         self.setLayout(layout)

#     def show_text(self):
#         # QLineEdit에서 텍스트 가져오기
#         entered_text = self.line_edit.text()
#         self.label.setText(f"You entered: {entered_text}")  # QLabel에 텍스트 표시

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = LineEditExample()
#     window.show()
#     sys.exit(app.exec_())


import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QTableView
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QStandardItemModel, QStandardItem
# class TableViewExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("QTableView Example")
#         self.setGeometry(100, 100, 400, 300)

#         # 레이아웃 생성
#         layout = QVBoxLayout()

#         # QTableView 및 모델 생성
#         self.table_view = QTableView()
#         self.model = QStandardItemModel(5, 3)  # 5행 3열

#         # 데이터 추가
#         for row in range(5):
#             for column in range(3):
#                 item = QStandardItem(f"Item {row}, {column}")
#                 self.model.setItem(row, column, item)

#         self.table_view.setModel(self.model)

#         # 셀 클릭 이벤트 연결
#         self.table_view.clicked.connect(self.cell_clicked)

#         # 레이아웃에 QTableView 추가
#         layout.addWidget(self.table_view)
#         self.setLayout(layout)

#     def cell_clicked(self, index):
#         # 클릭된 셀의 데이터 가져오기
#         clicked_data = self.model.item(index.row(), index.column()).text()
#         print(f"Clicked data: {clicked_data}")

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = TableViewExample()
#     window.show()
#     sys.exit(app.exec_())

# import sys
# from PyQt5.QtWidgets import  QPushButton
# from PyQt5.QtCore import Qt

# class TableViewExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("QTableView Example")
#         self.setGeometry(100, 100, 400, 300)

#         # 레이아웃 생성
#         layout = QVBoxLayout()

#         # QTableView 및 모델 생성
#         self.table_view = QTableView()
#         self.model = QStandardItemModel(5, 3)  # 5행 3열

#         # 데이터 추가
#         for row in range(5):
#             for column in range(3):
#                 item = QStandardItem(f"Item {row}, {column}")
#                 self.model.setItem(row, column, item)

#         self.table_view.setModel(self.model)

#         # 삭제 버튼 생성
#         self.delete_button = QPushButton("Delete Selected Column")
#         self.delete_button.clicked.connect(self.delete_column)  # 버튼 클릭 시 delete_column 메서드 호출

#         # 레이아웃에 위젯 추가
#         layout.addWidget(self.table_view)
#         layout.addWidget(self.delete_button)
#         self.setLayout(layout)

#     def delete_column(self):
#         # 선택된 셀의 인덱스를 가져오기
#         index = self.table_view.currentIndex()
#         if index.isValid():
#             column = index.row()  # 선택된 셀의 열 인덱스
#             self.model.removeRow(column)  # 해당 열 삭제
#             print(f"Deleted column {column}.")
#         else:
#             print("No column selected.")

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = TableViewExample()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QDial, QPushButton

# class DialExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Transparent QDial Example")
#         self.setGeometry(100, 100, 300, 200)

#         # 레이아웃 생성
#         layout = QVBoxLayout()

#         # QDial 생성
#         self.dial = QDial()
#         self.dial.setRange(0, 100)  # 다이얼의 범위 설정
#         self.dial.setValue(50)  # 초기 값 설정
        
#         # 다이얼의 배경을 투명하게 설정하는 QSS
#         self.dial.setStyleSheet("QDial { background: rgba(255, 255, 255, 50.0); }")

#         # 레이아웃에 QDial 추가
#         layout.addWidget(self.dial)
#         self.setLayout(layout)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = DialExample()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QDial

# class ContinuousDial(QDial):
#     def __init__(self, *args, **kwargs):
#         super().__init__(*args, **kwargs)
#         self.setRange(0, 100)  # 원하는 범위 설정
#         self.current_value = 0

#     def setValue(self, value):
#         # 값이 최대를 초과할 경우 처리
#         if value > self.maximum():
#             self.current_value += value - self.maximum()
#             super().setValue(self.current_value)
#         else:
#             super().setValue(value)

#     def value(self):
#         return self.current_value + super().value()

# class DialExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Continuous Dial Example")
#         self.setGeometry(100, 100, 300, 200)

#         # 레이아웃 생성
#         layout = QVBoxLayout()

#         # ContinuousDial 생성
#         self.dial = ContinuousDial()
#         self.dial.setValue(0)  # 초기 값 설정
#         self.dial.valueChanged.connect(self.update_label)  # 값 변경 시 레이블 업데이트

#         # QLabel로 다이얼의 값을 표시
#         self.label = QLabel("Value: {}".format(self.dial.value()))

#         # 레이아웃에 QDial과 QLabel 추가
#         layout.addWidget(self.dial)
#         layout.addWidget(self.label)
#         self.setLayout(layout)

#     def update_label(self):
#         # 레이블 업데이트
#         self.label.setText("Value: {}".format(self.dial.value()))

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = DialExample()
#     window.show()
#     sys.exit(app.exec_())

# import sys
# import numpy as np
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QDial
# from PyQt5.QtCore import Qt, QPoint, QRect
# from PyQt5.QtGui import QPainter, QColor

# class RotatingBar(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.value = 0  # 초기값 설정
#         self.setMinimumSize(300, 300)

#     def setValue(self, value):
#         self.value = value
#         self.update()  # 위젯을 다시 그리도록 요청

#     def paintEvent(self, event):
#         painter = QPainter(self)
#         painter.setRenderHint(QPainter.Antialiasing)

#         # 중심점 계산
#         center = QPoint(self.width() // 2, self.height() // 2)
#         length = 100  # 막대의 길이
#         angle = (self.value / 100) * 360  # 값에 따라 각도 계산

#         # 막대 색상 설정
#         painter.setPen(QColor(0, 100, 255))
#         # np.cos와 np.sin 결과를 int로 변환
#         end_point = QPoint(int(center.x() + length * np.cos(np.radians(angle - 90))),
#                            int(center.y() + length * np.sin(np.radians(angle - 90))))
#         painter.drawLine(center, end_point)

# class DialExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Dial with Rotating Bar Example")
#         self.setGeometry(100, 100, 400, 400)

#         # 레이아웃 생성
#         layout = QVBoxLayout()

#         # QDial 생성
#         self.dial = QDial()
#         self.dial.setRange(0, 100)  # 범위 설정
#         self.dial.setValue(0)  # 초기값 설정
#         self.dial.setNotchesVisible(True)  # 눈금 표시

#         # RotatingBar 생성
#         self.rotating_bar = RotatingBar()

#         # QLabel로 다이얼의 값을 표시
#         self.label = QLabel("Value: {}".format(self.dial.value()))

#         # 레이아웃에 QDial, RotatingBar, QLabel 추가
#         layout.addWidget(self.dial)
#         layout.addWidget(self.rotating_bar)
#         layout.addWidget(self.label)
#         self.setLayout(layout)

#         # 다이얼의 위치를 RotatingBar의 중앙으로 설정
#         self.dial.setParent(self.rotating_bar)
#         self.dial.move((self.rotating_bar.width() - self.dial.width()) // 2,
#                        (self.rotating_bar.height() - self.dial.height()) // 2)

#         # 다이얼의 값 변경 시 막대 업데이트
#         self.dial.valueChanged.connect(self.update_bar)

#     def update_bar(self):
#         # 막대의 값을 다이얼의 값으로 설정
#         self.rotating_bar.setValue(self.dial.value())
#         self.label.setText("Value: {}".format(self.dial.value()))

#     def resizeEvent(self, event):
#         # 다이얼의 위치를 RotatingBar의 중앙으로 재조정
#         self.dial.move((self.rotating_bar.width() - self.dial.width()) // 2,
#                        (self.rotating_bar.height() - self.dial.height()) // 2)
#         super().resizeEvent(event)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = DialExample()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# import numpy as np
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QDial
# from PyQt5.QtGui import QPainter, QColor
# from PyQt5.QtCore import Qt, QPoint

# class VerticalBarGraph(QWidget):
#     def __init__(self, data):
#         super().__init__()
#         self.data = data  # 그래프에 표시할 데이터
#         self.setMinimumSize(300, 400)  # 최소 크기 설정

#     def paintEvent(self, event):
#         painter = QPainter(self)
#         painter.setRenderHint(QPainter.Antialiasing)

#         # 그래프의 크기와 범위 설정
#         width = self.width() - 40  # 여백을 감안한 너비
#         height = self.height() - 40  # 여백을 감안한 높이
#         max_value = max(self.data) if self.data else 1  # 최대값 계산

#         # 축 그리기
#         painter.setPen(QColor(0, 0, 0))
#         painter.drawLine(20, height + 20, 20, 20)  # Y축
#         painter.drawLine(20, height + 20, width + 20, height + 20)  # X축

#         # 그래프 그리기
#         bar_width = width / len(self.data)  # 막대 너비 계산
#         for i, value in enumerate(self.data):
#             bar_height = (value / max_value) * height  # 비율에 따라 높이 계산
#             painter.setBrush(QColor(0, 100, 255))  # 막대 색상 설정
#             # float 결과를 int로 변환하여 drawRect에 전달
#             x = int(20 + i * bar_width)
#             y = int(height + 20 - bar_height)
#             w = int(bar_width - 2)
#             h = int(bar_height)
#             painter.drawRect(x, y, w, h)

# class GraphExample(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Vertical Bar Graph Example")
#         self.setGeometry(100, 100, 400, 400)

#         # 데이터 예시
#         self.data = [10, 20, 30, 25, 15, 40, 35]

#         # VerticalBarGraph 위젯 생성
#         self.graph = VerticalBarGraph(self.data)

#         # 레이아웃 설정
#         layout = QVBoxLayout()
#         layout.addWidget(self.graph)
#         self.setLayout(layout)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = GraphExample()
#     window.show()
#     sys.exit(app.exec_())


# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QDialog, QLabel

# class MainWindow(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("메인 윈도우")
#         self.setGeometry(100, 100, 300, 200)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # 버튼 생성
#         self.button = QPushButton("새 창 열기")
#         self.button.clicked.connect(self.open_dialog)

#         layout.addWidget(self.button)
#         self.setLayout(layout)

#     def open_dialog(self):
#         # 대화 상자 생성 및 표시
#         dialog = CustomDialog(self)
#         dialog.exec_()  # 모달 대화 상자 실행

# class CustomDialog(QDialog):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("대화 상자")
#         self.setGeometry(150, 150, 200, 100)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # 라벨 추가
#         self.label = QLabel("이것은 대화 상자입니다.")
#         layout.addWidget(self.label)

#         self.setLayout(layout)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     main_window = MainWindow()
#     main_window.show()
#     sys.exit(app.exec_())

# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QDialog, QLabel
# from PyQt5.QtCore import pyqtSignal
# class MainWindow(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("메인 윈도우")
#         self.setGeometry(100, 100, 300, 200)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # 버튼 생성
#         self.button = QPushButton("새 비모달 창 열기")
#         self.button.clicked.connect(self.open_dialog)

#         layout.addWidget(self.button)
#         self.setLayout(layout)

#         self.Motor_instancelist = []  # 대화 상자 인스턴스를 저장할 리스트

#     def open_dialog(self):
#         motor = SubWindowClass(self)  # 새로운 대화 상자 인스턴스 생성
#         motor.closed.connect(lambda: self.remove_instance(motor))  # 대화 상자가 닫힐 때 호출될 메서드 연결
#         motor.show()  # 대화 상자 표시
#         self.Motor_instancelist.append(motor)  # 인스턴스를 리스트에 추가

#     def remove_instance(self, motor):
#         # 닫힌 대화 상자를 리스트에서 제거
#         if motor in self.Motor_instancelist:
#             self.Motor_instancelist.remove(motor)
#             print("대화 상자가 닫혔습니다.")

# class SubWindowClass(QDialog):
#     closed = pyqtSignal()  # 사용자 정의 시그널

#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("비모달 대화 상자")
#         self.setGeometry(150, 150, 200, 100)

#         # 레이아웃 설정
#         layout = QVBoxLayout()

#         # 라벨 추가
#         self.label = QLabel("이것은 비모달 대화 상자입니다.")
#         layout.addWidget(self.label)

#         self.setLayout(layout)

#     def closeEvent(self, event):
#         self.closed.emit()  # 대화 상자가 닫힐 때 시그널 발생
#         event.accept()  # 이벤트 수용

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     main_window = MainWindow()
#     main_window.show()
#     sys.exit(app.exec_())


# class Aplication_Config:
#     def __init__(self):
#         self.update = False
#         self.buffer = []  # 빈 리스트로 초기화
#         self.maxMechanicalSpeed = 0  # uint32_t
#         self.maxReadableCurrent = 0  # float_t
#         self.nominalCurrent = 0  # float_t
#         self.nominalVoltage = 0  # uint16_t
#         self.driveType = 0  # uint8_t
#         self.padding = 0  # uint8_t

# def update_config(config):
#     # config의 속성을 수정
#     config.maxMechanicalSpeed = 100
#     config.maxReadableCurrent = 5.5
#     config.nominalCurrent = 2.5
#     config.nominalVoltage = 220
#     config.driveType = 1  # 예시: 특정 드라이브 타입
#     config.padding = 0

# # 인스턴스 생성
# app_config = Aplication_Config()

# # 초기 상태 출력
# print("Before update:")
# print("maxMechanicalSpeed:", app_config.maxMechanicalSpeed)
# print("maxReadableCurrent:", app_config.maxReadableCurrent)

# # 함수 호출하여 속성 수정
# update_config(app_config)

# # 수정된 상태 출력
# print("After update:")
# print("maxMechanicalSpeed:", app_config.maxMechanicalSpeed)
# print("maxReadableCurrent:", app_config.maxReadableCurrent)



# import sys
# from PyQt5 import QtWidgets, QtCore
# import sys
# from PyQt5.QtWidgets import *
# from PyQt5 import uic
# from PyQt5.QtGui import QStandardItemModel, QStandardItem
# from PyQt5.QtCore import Qt, QPoint, QRect
# from PyQt5.QtGui import QPainter, QColor, QFont  
# from PyQt5.QtCore import QTimer, pyqtSignal
# import numpy as np
# import time
# from canable import *
# from backend import *
# from motor_control import *

# class MainWindow(QtWidgets.QMainWindow):
#     def __init__(self):
#         super(MainWindow, self).__init__()
#         self.setWindowTitle("Dock Widget Example")
#         self.setGeometry(100, 100, 800, 600)

#         # Dock Widget 생성
#         dock_widget = QtWidgets.QDockWidget("Dockable", self)
#         dock_widget.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea)

#         # 위젯 추가
#         dock_content = QtWidgets.QTextEdit()
#         dock_widget.setWidget(dock_content)

#         # Dock Widget을 메인 윈도우에 추가
#         self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock_widget)

# if __name__ == '__main__':
#     app = QtWidgets.QApplication(sys.argv)
#     main_window = MainWindow()
#     main_window.show()
#     sys.exit(app.exec_())


