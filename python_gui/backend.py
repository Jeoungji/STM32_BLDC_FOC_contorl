from canable import *
from main_window import *
import queue
from motor_control import *
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QStandardItemModel, QStandardItem
import numpy as np
import time
import queue

class Reciver(QThread):
    def __init__(self, canable_):
        super().__init__()
        self.canable = canable_
        
    update_signal = pyqtSignal(can.Message)
    
    def run(self):
        while True:
            message = self.canable.Read()
            if message != None:
                self.update_signal.emit(message)

class BackEnd():
    
    CAN_txmodel = QStandardItemModel(0, 3)
    CAN_txmodel.setHorizontalHeaderLabels(["ID", "DLC", "data","",""])
    
    CAN_rxmodel = QStandardItemModel(0, 5)
    CAN_rxmodel.setHorizontalHeaderLabels(["Timestamp", "channel","Rx/Tx", "ID", "DLC", "data",""])
    CAN_rx_lengths = 10
    CAN_auto_scroll = False
        
    def __init__(self, dev = None, win = None):
        
        self.cantableupdateT = 100
        self.motordataupdateT = 200
        self.motortableupdateT = 500
        
        self.canable = dev
        self.window = win
        self.SearchDevList = list()
        self.receive_queue = queue.Queue()
        self.data_list = list()
        
        ## motor Search variable
        self.MotorSearch_startpoint = 10
        self.MotorSearch_endpoint = 50
        self.MotorSearch_list = list()
        self.__MotorSearch_list = False
        self.MotorSearch_timer = QTimer()
        self.MotorSearch_timer.timeout.connect(self.MotorSearch_SendResponse)
        self.MotorSearch_q = queue.Queue()
        
        self.Motor_instancelist = list()
        
        self.canreciver = Reciver(self.canable)
        self.canreciver.update_signal.connect(self.ReadMessageCallback)
  
        self.canguiupdatetimer = QTimer()
        self.canguiupdatetimer.timeout.connect(self.CAN_update_rxtable)
        self.aggregateID = False
        self.aggregateHeader = False
    
    def __del__(self):
        self.close()
    
    def close(self):
        self.MotorSearch_timer.stop()
        self.canguiupdatetimer.stop()
        for m in self.Motor_instancelist:
            if m is not None:
                m.close()
        self.Motor_instancelist.clear()
        self.DevDisConnect()
        
    def CAN_Rx_Change_AggregateID(self, state):
        self.aggregateID = state
        self.CAN_rxmodel.removeRows(0, self.CAN_rxmodel.rowCount())
        if self.aggregateID or self.aggregateHeader:
            self.CAN_rx_lengths = 32767
        else:
            self.CAN_rx_lengths = self.window.CAN_rx_length_spinbox.value()
        
    def CAN_Rx_Change_AggregateHeader(self, state):
        self.aggregateHeader = state
        self.CAN_rxmodel.removeRows(0, self.CAN_rxmodel.rowCount())
        if self.aggregateID or self.aggregateHeader:
            self.CAN_rx_lengths = 32767
        else:
            self.CAN_rx_lengths = self.window.CAN_rx_length_spinbox.value()
    
    def Send_motor(self, com:str, data=-1):
        if not self.canable.GetConnectedStatus():
            return
        if com == "power":
            for m in self.Motor_instancelist:
                if m.power:
                    m.Set_PowerOFF()
                else:
                    m.Set_PowerON()
        elif com == "pos":
            for m in self.Motor_instancelist:
                m.Set_Position(data * np.pi / 180)
        
    def start_motor(self, id):
        if len(self.Motor_instancelist) != 0:
            for instance in self.Motor_instancelist:
                if instance.this_id == id:
                    return
        motor = SubWindowClass(self, id)
        motor.closed.connect(lambda: self.remove_instance(motor)) 
        motor.show()
        self.Motor_instancelist.append(motor)
        
    def remove_instance(self, motor):
        if motor in self.Motor_instancelist:
            self.Motor_instancelist.remove(motor)
    
    def set_window_instance(self, win):
        self.window = win
        
    def AddDevList(self, dev):
        self.SearchDevList.append(dev)
        
    def DevConnect(self, deviceName:str='canable gs_usb'):
        if self.canable.GetConnectedStatus():
            return self.canable.GetConnectedStatus()
        if self.SearchDevList != None:
            for dev in self.SearchDevList:
                if (dev[0] == deviceName):
                    self.canable.Connect(dev[1], dev[2])
                    self.canguiupdatetimer.start(self.cantableupdateT)
                    self.canreciver.start()
                    return True
        return False
    
    def DevDisConnect(self):
        self.__MotorSearch_list = False
        for m in self.Motor_instancelist:
            m.__del__()
        time.sleep(0.1)
        self.canreciver.terminate()
        self.canguiupdatetimer.stop()
        return self.canable.DisConnect()
    
    def DevStatus(self):
        return self.canable.GetConnectedStatus()
    
    def SetDeviceBaudrate(self, baudrate_):
        self.canable.SetBaudrate(int(baudrate_))
        
    def SearchDevice(self):
        self.SearchDevList = self.canable.scan_devices()
        return self.SearchDevList
    
    def ReadMessageCallback(self, message):
        if self.__MotorSearch_list:
            if message.dlc == 4 and message.arbitration_id + 1 == message.data[3]:
                self.MotorSearch_list.append(str(hex(message.arbitration_id + 1)))
                self.window.MotorSearch_model.setStringList(self.MotorSearch_list)
        else:
            self.receive_queue.put(message)
            for m in self.Motor_instancelist:
                m.CAN_data_Recive(message)
    
    def CAN_Rx_Lengths(self, value):
        self.CAN_rx_lengths = value
    
    def CAN_update_rxtable(self):
        while not self.receive_queue.empty():
            message = self.receive_queue.get()
            Xx = "Rx"
            if not message.is_rx:
                Xx = "Tx"
                
            mdata = ""
            for value in message.data:
                mdata += str(hex(value))
                mdata += " "
            data = [message.timestamp, message.channel, Xx, str(hex(message.arbitration_id)), message.dlc, mdata, "Save"]
            header = mdata.split()[:3]
            
            addData = True
            same_id = False
            same_header = False
            roww = -1
            if self.aggregateID or self.aggregateHeader:
                for row in range(self.CAN_rxmodel.rowCount()):

                    item_id = int(self.CAN_rxmodel.item(row, 3).text()[2:], 16)
                    item_header = self.CAN_rxmodel.item(row,5).text().split()[:3]
                    
                    if self.aggregateID and item_id == message.arbitration_id:
                        same_id = True
                    if self.aggregateHeader and item_header == header:
                        same_header = True
                    
                    if self.aggregateHeader and self.aggregateID and same_id and same_header:
                        addData = False
                        roww = row
                        break
                    elif self.aggregateHeader and not self.aggregateID and same_header:
                        self.CAN_rxmodel.setItem(row, 3, QStandardItem(str(hex(message.arbitration_id))))
                        addData = False
                        roww = row
                        break
                    elif self.aggregateID and not self.aggregateHeader and same_id:
                        addData = False
                        roww = row
                        break

            if addData:
                self.CAN_rxmodel.appendRow([QStandardItem(str(value)) for value in data])
            else:
                self.CAN_rxmodel.setItem(roww, 0, QStandardItem(str(message.timestamp)))
                self.CAN_rxmodel.setItem(roww, 4, QStandardItem(str(message.dlc)))
                self.CAN_rxmodel.setItem(roww, 5, QStandardItem(mdata))
                
            if self.CAN_rxmodel.rowCount() > self.CAN_rx_lengths:
                self.CAN_rxmodel.removeRow(0)
                
            if self.window is not None and self.CAN_auto_scroll:
                self.window.CAN_main_tableView.scrollToBottom()
            
    def CAN_Message_Send(self, id, dlc, message):
        try:
            strmessage = ""
            for i in range(dlc):
                strmessage += (message[i] + " ")

            duplicate_found = False
            for row in range(self.CAN_txmodel.rowCount()):
                item_id = int(self.CAN_txmodel.item(row, 0).text(), 16)
                item_dlc = int(self.CAN_txmodel.item(row, 1).text())
                item_message = self.CAN_txmodel.item(row, 2).text()
                if item_id == id and item_dlc == dlc and item_message == strmessage:
                    duplicate_found = True
                    break
                
            if not duplicate_found:
                strdata = [hex(id), dlc, strmessage, "Send", "Clear"]
                self.CAN_txmodel.appendRow([QStandardItem(str(value)) for value in strdata])
            
            self.CAN_send_data(id, dlc, message)
        except:
            print("value error")
    
    def CAN_Table_Send(self, index):
        if index.column() == 3:
            self.CAN_send_data(int(self.CAN_txmodel.item(index.row(), 0).text()[2:],16),
                                       int(self.CAN_txmodel.item(index.row(), 1).text()),
                                       [hex_value for hex_value in self.CAN_txmodel.item(index.row(), 2).text().split()])
        elif index.column() == 4:
            if index.isValid():
                row = index.row()
                self.CAN_txmodel.removeRow(row)
    
    def CAN_send_data(self, id, dlc, data):
        if not self.canable.GetConnectedStatus():
            return
        message = list()
        for i in range(dlc):
            message.append(int(data[i][2:], 16))
        self.canable.Send(id, dlc, message)
        
    def CAN_Table_Save(self, index):
        if index.column() == 6:
            print("save")
        
            
    def MotorSearch_SendReady(self):
        if not self.canable.GetConnectedStatus():
            self.window.error_event('Device Error','장치가 연결되지 않았습니다.')
            return
        self.MotorSearch_list.clear()
        self.window.MotorSearch_model.setStringList(self.MotorSearch_list)
        self.__MotorSearch_list = True
        for i in range(self.MotorSearch_startpoint, self.MotorSearch_endpoint+1):
            self.MotorSearch_q.put(i)      
        self.MotorSearch_timer.start(20)
      
    def MotorSearch_SendResponse(self):
        if self.MotorSearch_q.empty():
            self.MotorSearch_timer.stop()
            self.__MotorSearch_list = False
            return
        id =self.MotorSearch_q.get()
        self.canable.Send(id, 3, [0,0,0])
        
    def MotorSearch_Set_Start_End_Point(self,start = -1, end = -1):
        if start != -1:
            self.MotorSearch_startpoint = start
        if end != -1:
            self.MotorSearch_endpoint = end