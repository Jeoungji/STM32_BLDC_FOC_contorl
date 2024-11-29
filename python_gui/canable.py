import usb
import can
import time



class CanableClass:
    def __init__(self):
        self.dev = None
        self.bus = None
        self.canable_connected = False
        self.last_received_message = None
        self.backend = None
        
        self.baudrate = 0

    def GetConnectedStatus(self):
        return self.canable_connected
    
    def scan_devices(self):
        devices = list(usb.core.find(find_all=True))
        ret = list()
        if len(devices) == 0:
            return None
        for device in devices:
            try:
                device_name = usb.util.get_string(device, device.iProduct) if device.iProduct else "N/A"
                ret.append([f"{device_name}", device.idVendor, device.idProduct])
                print(ret)
            except Exception as e:
                print(f"Error retrieving information for device idVendor={hex(device.idVendor)}, idProduct={hex(device.idProduct)}: {e}")
        
        if len(ret) == 0:
            return None
        return ret
        
    def set_backend_instance(self, _backend):
        self.backend = _backend
        
    def SetBaudrate(self, baudrate_):
        if type(baudrate_) == int:
            self.baudrate = baudrate_

    def Connect(self, idvendor_=0x1D50, idproduct_=0x606F, autoRead = True):
        # idVendor=0x1D50, idProduct=0x606F
        if not self.canable_connected and self.baudrate != 0:
            self.dev = usb.core.find(idVendor=idvendor_, idProduct=idproduct_)
            if self.dev is None:
                print("can't find")
                return False

            self.bus = can.interface.Bus(interface="gs_usb", channel=self.dev.product, index=0, bitrate=self.baudrate)
            self.canable_connected = True
            self.last_received_message = None
            return True
        
    def DisConnect(self):
        # 연결 끊기
        self.last_received_message = None
        if self.canable_connected:
            self.canable_connected = False
            self.bus.shutdown()
            self.dev.reset()
            self.bus = None
            self.dev = None
            print("disconnect")
            return True
        return False
        
    def Send(self, id:int, len:int, data:list):
        if self.canable_connected:
            message = can.Message(arbitration_id=id, dlc=len, data=data, is_rx=False, is_extended_id=False)
            try:
                self.bus.send(message)
            except can.CanError:
                print("Message could not be sent.")
        else:
            print("didn't connect")
                
    def Read(self, timeout = 0):
        if not self.canable_connected:
            return None
        starttime = time.time() / 1000
        while True:
            received_message = self.bus.recv()
            if not (received_message == None or not received_message.is_rx or received_message.dlc == 0):
                if self.last_received_message is None or (
                    received_message.arbitration_id != self.last_received_message.arbitration_id or
                    received_message.data != self.last_received_message.data or
                    len(received_message.data) != len(self.last_received_message.data)
                ):
                    self.last_received_message = received_message
                    return received_message
            
            if time.time() / 1000 - starttime >= timeout:
                return None
    
    def ReadWaitData(self, id, dlc, data, timeout = 0):
        print(id, " ", dlc, " ", data, " ", timeout)
        if not self.canable_connected:
            return False
        starttime = time.time() / 1000
        while True:
            message = self.bus.recv()
            if message != None:
                print(message)
                if message.arbitration_id == id and message.dlc == dlc:
                    success = True
                    for value in range(0,dlc):
                        if message.data[value] != data[value]:
                            success = False
                    if success:
                        return True
            if ((time.time() / 1000.0) - starttime) >= timeout:
                return False
                
    
    def receive_callback(self):
        self.strattime =time.time() *1000
        received_message = None
        while self.canable_connected:
            received_message = self.bus.recv()  # 수신 대기
            if self.check_prev_message(received_message):
                self.last_received_message = received_message
                self.backend.ReadMessageCallback(received_message)

                
                
    def check_prev_message(self, curmessage):
        if curmessage == None or curmessage.dlc == 0:
            return False
        if self.last_received_message is None:
            return True
        if (curmessage.arbitration_id != self.last_received_message.arbitration_id or
            curmessage.dlc != self.last_received_message.dlc or
            curmessage.data != self.last_received_message.data or
            curmessage.is_rx != self.last_received_message.is_rx):
            return True
        return False