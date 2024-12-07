import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import QStringListModel, Qt
import numpy as np
import time
from canable import *
from backend import *
from Sub_window import *

curr_path = os.path.dirname(os.path.abspath(__file__))
form_class = uic.loadUiType(curr_path + "\Can.ui")[0]

class CustomComboBox(QComboBox):
    def __init__(self, parent=None, _backend = None) :
        super().__init__(parent)
        self.setGeometry(435, 10, 130, 20)
        self.backend = _backend
        
    def showPopup(self):
        self.clear()
        self.lis = self.backend.SearchDevice()
        if self.lis == None:
            return
        for dev in self.lis:
            self.addItem(dev[0])
        super().showPopup()

#화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, form_class) :
    
    closed = pyqtSignal()
    
    def __init__(self, _canable, _backend) :
        super().__init__()
        self.setupUi(self)
        self.canable = _canable
        self.backend = _backend
        self.current_device = None

        self.connect_button_state = False
        
        self.baudrate_combo_box.addItems([str(10000), str(20000), str(50000), str(83333), str(100000), str(125000), str(250000), str(500000), str(800000), str(1000000)])
        self.baudrate_combo_box.setCurrentText("1000000")
        self.backend.SetDeviceBaudrate("1000000")

        
        self.canable_connect_botton.clicked.connect(self.Canable_Toggle)
        self.device_combo_box = CustomComboBox(self, _backend = self.backend)
        self.device_combo_box.currentIndexChanged.connect(self.device_selected)
       

        self.CAN_main_tableView.setModel(self.backend.CAN_rxmodel)
        self.CAN_main_tableView.setColumnWidth(0, 100)
        self.CAN_main_tableView.setColumnWidth(1, 70)
        self.CAN_main_tableView.setColumnWidth(2, 40)
        self.CAN_main_tableView.setColumnWidth(3, 50)
        self.CAN_main_tableView.setColumnWidth(4, 40)
        self.CAN_main_tableView.setColumnWidth(5, 250)
        self.CAN_main_tableView.setColumnWidth(6, 50)
        self.CAN_main_tableView.clicked.connect(self.CAN_Recive_re)
        
        self.CAN_rx_length_spinbox.valueChanged.connect(self.CAN_Rx_Lengths)
        self.CAN_rx_aggregate_checkbox.stateChanged.connect(self.CAN_Rx_Change_AggregateID)
        self.CAN_rx_header_aggregate_checkbox.stateChanged.connect(self.CAN_Rx_Change_AggregateHeader)
        
        self.CAN_tx_tableView.setModel(self.backend.CAN_txmodel)
        self.CAN_tx_tableView.setColumnWidth(0, 50)
        self.CAN_tx_tableView.setColumnWidth(1, 40)
        self.CAN_tx_tableView.setColumnWidth(2, 250)
        self.CAN_tx_tableView.setColumnWidth(3, 60)
        self.CAN_tx_tableView.setColumnWidth(4, 60)
        self.CAN_tx_single_button.clicked.connect(self.CAN_transmit)
        self.CAN_tx_tableView.clicked.connect(self.CAN_trnsmit_re)
        
        self.MotorSearch_startnum.valueChanged.connect(self.motor_search_start_update)
        self.MotorSearch_endnum.valueChanged.connect(self.motor_search_end_update)
        self.MotorSearch_start_button.clicked.connect(self.motor_search_st)
        self.MotorSearch_model = QStringListModel()
        self.MotorSearch_list.setModel(self.MotorSearch_model)
        self.MotorSearch_list.clicked.connect(self.motor_control_app_excute)


        
    def CAN_trnsmit_re(self, index):
        self.backend.CAN_Table_Send(index)
    
    def CAN_Recive_re(self, index):
        self.backend.CAN_Table_Save(index)
        

    def CAN_transmit(self):
            id = int(self.CAN_tx_id.text(), 16)
            dlc = self.CAN_tx_dlc.value()
            if id == 0:
                self.error_event('ID Error', 'CAN ID는 0이 될 수 없습니다.')
                return
            message = [hex(int(self.CAN_tx_0.text(), 16)), hex(int(self.CAN_tx_1.text(), 16)),
                        hex(int(self.CAN_tx_2.text(), 16)), hex(int(self.CAN_tx_3.text(), 16)),
                        hex(int(self.CAN_tx_4.text(), 16)), hex(int(self.CAN_tx_5.text(), 16)),
                        hex(int(self.CAN_tx_6.text(), 16)), hex(int(self.CAN_tx_7.text(), 16)) ]
            self.backend.CAN_Message_Send(id, dlc, message)
            
            

    def CAN_Rx_Change_AggregateID(self):
        self.backend.CAN_Rx_Change_AggregateID(self.CAN_rx_aggregate_checkbox.isChecked())
        
    def CAN_Rx_Change_AggregateHeader(self):
        self.backend.CAN_Rx_Change_AggregateHeader(self.CAN_rx_header_aggregate_checkbox.isChecked())

    def motor_control_app_excute(self, index):
        motor_name = int(self.MotorSearch_model.data(index, Qt.DisplayRole)[2:],16)
        self.backend.start_motor(motor_name)
        
    def motor_search_st(self):
        self.backend.MotorSearch_SendReady()
        
    def motor_search_start_update(self, value):
        self.backend.MotorSearch_Set_Start_End_Point(value, -1)
        if self.MotorSearch_endnum.value() < value:
            self.MotorSearch_endnum.setValue(value)
        
    def motor_search_end_update(self, value):
        self.backend.MotorSearch_Set_Start_End_Point(-1, value)
        if self.MotorSearch_startnum.value() > value:
            self.MotorSearch_startnum.setValue(value)
        
    def CAN_Rx_Lengths(self, value):
        self.backend.CAN_Rx_Lengths(value)
        
    def device_selected(self):
        self.current_device = self.device_combo_box.currentText()

    def Canable_Toggle(self):
        if self.current_device == None:
            return
        if not self.connect_button_state:
            if self.backend.DevConnect(self.current_device):
                self.connect_button_state = True
                self.canable_connect_botton.setText("Connected")
                self.canable_connect_botton.setStyleSheet("background-color: rgb(220, 220, 220);")
                return
        else:
            self.backend.DevDisConnect()
            self.current_device = None
            self.device_combo_box.clear()
            self.canable_connect_botton.setText("Connect")
            self.canable_connect_botton.setStyleSheet("background-color: rgb(225, 225, 225);")
        self.connect_button_state = False
        
    def MotorContorl_powerbutton_switching(self, power):
        if power:
            self.MotorControl_poweron_button.setText("PowerOn")
            self.MotorControl_poweron_button.setStyleSheet("background-color: rgb(220, 220, 220);")
        else:
            self.MotorControl_poweron_button.setText("PowerOff")
            self.MotorControl_poweron_button.setStyleSheet("background-color: rgb(225, 225, 225);")
    
    def error_event(self, title, message) :
        QMessageBox.critical(self,title,message)
        
    def closeEvent(self, event):
        print("Close")
        self.closed.emit()
        event.accept()
        self.backend.close()


if __name__ == "__main__" :
    app = QApplication(sys.argv)
    canable = CanableClass()
    backend = BackEnd(dev = canable)
    myWindow = WindowClass(canable, backend) 
    canable.set_backend_instance(backend)
    backend.set_window_instance(myWindow)
    
    myWindow.show()
    app.exec_()
    canable.DisConnect()
    print("done")