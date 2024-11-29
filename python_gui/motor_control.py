from MC_Enum import *
import struct
import time
import queue
from canable import *
from PID_gain_profile import *
    
class Motor_Control:
    
    position_limit = [0xFFFFFFFF,0xFFFFFFFF]
    velocity_limit = [0xFFFFFFFF,0xFFFFFFFF]
    iq_limit = [0xFFFFFFFF,0xFFFFFFFF]
    
    vbus = 0
    temperature = 0
    power = 0
    read_string = False
    
    motor_parameter = Motor_Param()
    motor_parameter_update = False
    
    mode = [False, U_Control_Mode.U_POS_VEL_TORQUE_MODE.value]
    
    def __init__(self, canable_, motorid = 1):
        self.canable = canable_
        self.motorid = motorid
        self.status = MC_State.IDLE
        self.faults = MC_NO_ERROR
        self.response = False
        self.data_queue = queue.Queue()
        self.position = 0
        self.velocity = 0
        self.iq = 0
        self.id = 0
        self.power = False
        self.encoder_speed = 0
        self.position_gain = PID()
        self.velocity_gain = PID()
        self.iq_gain = PID()
        self.id_gain = PID()
        self.reset_signal = False
    
    def __create_byte_list(self, command, typeID = 0, regID = 0):
        if isinstance(command, Command):
            command = command.value

        if not (0 <= command and command <= 15):
            raise ValueError("command must be between 0 and 15")
        if not (TYPE_DATA_SEG_END <= typeID and typeID <= TYPE_DATA_SEG_BEG):
            raise ValueError("typeID must be between 0 and 15")
        if not (0 <= regID and regID <= 65535):
            raise ValueError("regID must be between 0 and 65535")

        byte0 = (command << 4) | typeID
        byte2 = (regID >> 8) & 0xFF
        byte1 = regID & 0xFF

        return [byte0, byte1, byte2]
    
    def __parse_byte_list(self, byte_list):
        if len(byte_list) != 3:
            raise ValueError("Input must be a list of exactly 3 bytes")

        byte0 = byte_list[0]
        byte1 = byte_list[1]
        byte2 = byte_list[2]

        command = (byte0 >> 4) & 0x0F
        typeID = byte0 & 0x0F
        regID = (byte2 << 8) | byte1

        return command, typeID, regID
    
    def Response(self):
        self.response = False
        header = self.__create_byte_list(Command.RESPONSE)
        self.canable.Send(self.motorid, 3, header)
        self.CanDataWaitProcess(Command.RESPONSE)
    
    def Read_Faults(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_32BIT, MC_REG_FAULTS_FLAGS)
        self.canable.Send(self.motorid, 3, header)
     
    def Start_motor(self):
        header = self.__create_byte_list(Command.START_MOTOR)
        self.canable.Send(self.motorid, 3, header)   
    
    def Stop_motor(self):
        header = self.__create_byte_list(Command.STOP_MOTOR)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Status(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_8BIT, MC_REG_STATUS)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_USER_Mode(self):
        header = self.__create_byte_list(Command.USER_GET, U_TYPE_DATA_8BIT, U_REG_MODE)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Position(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_32BIT, MC_REG_CURRENT_POSITION)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Velocity(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_32BIT, MC_REG_SPEED_MEAS)
        self.canable.Send(self.motorid, 3, header)
    
    def Read_Velocity_Ref(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_32BIT, MC_REG_SPEED_REF)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Velocity_F(self):
        header = self.__create_byte_list(Command.USER_GET, U_TYPE_DATA_32BIT, MC_REG_SPEED_MEAS)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Iq(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_MEAS)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Iq_F(self):
        header = self.__create_byte_list(Command.USER_GET, U_TYPE_DATA_32BIT, MC_REG_I_Q_MEAS)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Id(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_MEAS)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Id_F(self):
        header = self.__create_byte_list(Command.USER_GET, U_TYPE_DATA_32BIT, MC_REG_I_D_MEAS)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Velocity_Limit(self):
        header = self.__create_byte_list(Command.USER_GET, U_TYPE_DATA_32BIT, U_REG_SPEED_LIMIT)
        self.canable.Send(self.motorid, 3, header)
        print("Read_Velocity_Limit")
        
    def Read_Iq_Limit(self):
        header = self.__create_byte_list(Command.USER_GET, U_TYPE_DATA_32BIT, U_REG_IQ_LIMIT)
        self.canable.Send(self.motorid, 3, header)
        print("Read_Iq_Limit")
        
    def Read_Temperature(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_HEATS_TEMP)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Power(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_32BIT, MC_REG_MOTOR_POWER)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Vbus(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_BUS_VOLTAGE)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Encoder_Speed(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_ENCODER_SPEED)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Stage(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_STRING, MC_REG_PWR_STAGE_NAME)
        self.canable.Send(self.motorid, 3, header)
        self.motor_parameter.power_stage[0] = False
        self.motor_parameter.power_stage[1] = ""
        
    def Read_MotorName(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_STRING, MC_REG_MOTOR_NAME)
        self.canable.Send(self.motorid, 3, header)
        self.motor_parameter.motor_name[0] = False
        self.motor_parameter.motor_name[1] = ""
        
    def Read_ApplicationConfig_reg_t(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_RAW, MC_REG_APPLICATION_CONFIG)
        self.canable.Send(self.motorid, 3, header)
        self.motor_parameter.application_config.update = False
        self.motor_parameter.application_config.buffer = []
        
    def Read_MotorConfig_reg_t(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_RAW, MC_REG_MOTOR_CONFIG)
        self.canable.Send(self.motorid, 3, header)
        self.motor_parameter.motor_config.update = False
        self.motor_parameter.motor_config.buffer = []
        
    def Read_FOCFwConfig_reg_t(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_RAW, MC_REG_FOCFW_CONFIG)
        self.canable.Send(self.motorid, 3, header)
        self.motor_parameter.focfw_config.update = False
        self.motor_parameter.focfw_config.buffer = []
        
    def Read_Position_Kp(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KP)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KP_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Position_Ki(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KI)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KI_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Position_Kd(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KD)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KD_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Velocity_Kp(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KP)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KP_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Velocity_Ki(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KI)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KI_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Iq_Kp(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KP)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KP_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Iq_Ki(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KI)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KI_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Id_Kp(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KP)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KP_DIV)
        self.canable.Send(self.motorid, 3, header)
        
    def Read_Id_Ki(self):
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KI)
        self.canable.Send(self.motorid, 3, header)
        header = self.__create_byte_list(Command.GET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KI_DIV)
        self.canable.Send(self.motorid, 3, header)
        
        
    def Reset(self):
        header = self.__create_byte_list(Command.USER_SET, 0, U_RESET)
        self.canable.Send(self.motorid, 3, header)
        
    def Set_PowerON(self):
        header = self.__create_byte_list(Command.START_MOTOR)
        self.canable.Send(self.motorid, 3, header)
        
    def Set_PowerOFF(self):
        header = self.__create_byte_list(Command.STOP_MOTOR)
        self.canable.Send(self.motorid, 3, header)
    
    def Ack_Faults(self):
        header = self.__create_byte_list(Command.FALULT_ACK)
        self.canable.Send(self.motorid, 3, header)
        
    def Set_USER_Mode(self, mode):
        header = self.__create_byte_list(Command.USER_SET, U_TYPE_DATA_8BIT, U_REG_MODE)
        header.append(mode)
        self.canable.Send(self.motorid, 4, header)
        
    def Set_Position(self, rad):
        header = self.__create_byte_list(Command.USER_SET, U_TYPE_DATA_32BIT, U_POSITION)
        float_bytes = struct.pack('<f', rad)
        self.canable.Send(self.motorid, 7, header + list(float_bytes))
        
    def Set_Velocity_Limit(self, vel_upper, vel_lower):
        print("Set_Velocity_Limit")
        header = self.__create_byte_list(Command.USER_SET, U_TYPE_DATA_32BIT, U_REG_SPEED_LIMIT)
        upper = struct.pack('<h', vel_upper)
        lower = struct.pack('<h', vel_lower)
        self.canable.Send(self.motorid, 7, header + list(upper) + list(lower))
        
    def Set_Iq_Limit(self, current_upper, current_lower):
        print("Set_Iq_Limit")
        header = self.__create_byte_list(Command.USER_SET, U_TYPE_DATA_32BIT, U_REG_IQ_LIMIT)
        upper = struct.pack('<h', current_upper)
        lower = struct.pack('<h', current_lower)
        self.canable.Send(self.motorid, 7, header + list(upper) + list(lower))
        
    def Set_Position_Kp(self, kp:int, kp_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KP)
        data = struct.pack('<h', kp)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KP_DIV)
        data = struct.pack('<h', kp_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Position_Ki(self, ki:int, ki_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KI)
        data = struct.pack('<h', ki)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KI_DIV)
        data = struct.pack('<h', ki_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Position_Kd(self, kd:int, kd_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KD)
        data = struct.pack('<h', kd)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_POSITION_KD_DIV)
        data = struct.pack('<h', kd_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Velocity_Kp(self, kp:int, kp_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KP)
        data = struct.pack('<h', kp)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KP_DIV)
        data = struct.pack('<h', kp_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Velocity_Ki(self, ki:int, ki_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KI)
        data = struct.pack('<h', ki)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_SPEED_KI_DIV)
        data = struct.pack('<h', ki_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Iq_Kp(self, kp:int, kp_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KP)
        data = struct.pack('<h', kp)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KP_DIV)
        data = struct.pack('<h', kp_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Iq_Ki(self, ki:int, ki_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KI)
        data = struct.pack('<h', ki)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_Q_KI_DIV)
        data = struct.pack('<h', ki_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Id_Kp(self, kp:int, kp_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KP)
        data = struct.pack('<h', kp)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KP_DIV)
        data = struct.pack('<h', kp_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Set_Id_Ki(self, ki:int, ki_div:int):
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KI)
        data = struct.pack('<h', ki)
        self.canable.Send(self.motorid, 5, header+list(data))
        header = self.__create_byte_list(Command.SET_DATA_ELEMENT, U_TYPE_DATA_16BIT, MC_REG_I_D_KI_DIV)
        data = struct.pack('<h', ki_div)
        self.canable.Send(self.motorid, 5, header+list(data))
        
    def Ramp_Velocity(self, speed=0xFFFF, duration=0xFFFF):
        """ velcoity ramp with SPEED UNIT & ms """
        print(speed, " ", duration)
        if self.mode[1] == U_Control_Mode.U_POS_VEL_TORQUE_MODE.value or self.mode[1] == U_Control_Mode.U_POSITION_TORQUE_MODE.value:
            return
        if duration == 0xFFFF:
            header = self.__create_byte_list(Command.USER_SET, U_TYPE_DATA_32BIT, MC_REG_SPEED_REF)
            byte4 = struct.pack('<i', speed)
            header += list(byte4)
            length = 4
        self.canable.Send(self.motorid, length+3, header)
        
    def Ramp_Torque(self, torque=0xFFFF, duration=0xFFFF):
        """ velcoity ramp with torque & ms """
        if self.mode[1] == U_Control_Mode.U_POS_VEL_TORQUE_MODE.value or self.mode[1] == U_Control_Mode.U_POSITION_TORQUE_MODE.value:
            return
        if duration == 0xFFFF:
            header = self.__create_byte_list(Command.USER_SET, U_TYPE_DATA_32BIT, MC_REG_CURRENT_REF)
            byte4 = struct.pack('<i', torque)
            header += list(byte4)
            length = 4
        self.canable.Send(self.motorid, 3+length, header)
        
    def CanDataProccesser(self, message):
        if message == None:
            return
        if message.arbitration_id != self.motorid - 1:
            return
        if len(message.data) < 3:
            return

        if message.data == RESETSIGNAL:
            self.reset_signal = True
            return
        
        header = self.__parse_byte_list(message.data[0:3])
        if header[0] == Command.RESPONSE.value:
            self.response = True

        elif header[0] == Command.GET_DATA_ELEMENT.value:
            if header[2] == MC_REG_STATUS:
                self.status = message.data[3]
                if self.status == MC_State.FAULT_NOW.value or self.status == MC_State.FAULT_OVER.value:
                    self.Read_Faults()
                elif self.status == MC_State.IDLE.value:
                    self.faults = MC_NO_ERROR
                    
            elif header[2] == MC_REG_FAULTS_FLAGS:
                self.faults = struct.unpack('<I', message.data[3:7])[0]
                
            elif header[2] == MC_REG_CURRENT_POSITION:
                self.position = struct.unpack('<f', message.data[3:7])[0]
                
            elif header[2] == MC_REG_SPEED_MEAS:
                self.velocity = float(struct.unpack('<i', message.data[3:7])[0]) / 60 * 10
            
            elif header[2] == MC_REG_I_Q_MEAS:
                self.iq = struct.unpack('<h', message.data[3:5])[0]
                
            elif header[2] == MC_REG_I_D_MEAS:
                self.id = struct.unpack('<h', message.data[3:5])[0]
                
            elif header[2] == MC_REG_BUS_VOLTAGE:
                self.vbus = struct.unpack('<h', message.data[3:5])[0]
                
            elif header[2] == MC_REG_HEATS_TEMP:
                self.temperature = struct.unpack('<h', message.data[3:5])[0]
                
            elif header[2] == MC_REG_MOTOR_POWER:
                self.power = struct.unpack('<f', message.data[3:7])[0]
                
            elif header[2] == MC_REG_ENCODER_SPEED:
                self.encoder_speed = (struct.unpack('<h', message.data[3:5])[0]) * self.motor_parameter.MAX_APPLICATION_SPEED_UNIT() / INT16_MAX
                
            elif header[2] == MC_REG_PWR_STAGE_NAME:
                self.Update_string(self.motor_parameter.power_stage, message.data)
                    
            elif header[2] == MC_REG_MOTOR_NAME:
                self.Update_string(self.motor_parameter.motor_name, message.data)

            elif header[2] == MC_REG_APPLICATION_CONFIG:
                if self.Update_struct(self.motor_parameter.application_config, message.data):
                    self.motor_parameter.application_config.Unpacked()
            elif header[2] == MC_REG_MOTOR_CONFIG:
                if self.Update_struct(self.motor_parameter.motor_config, message.data):
                    self.motor_parameter.motor_config.Unpacked()
            elif header[2] == MC_REG_FOCFW_CONFIG:
                if self.Update_struct(self.motor_parameter.focfw_config, message.data):
                    self.motor_parameter.focfw_config.Unpacked()
            elif header[2] == MC_REG_POSITION_KP:
                self.position_gain.Kp[1] = struct.unpack('<h', message.data[3:5])[0]
                self.position_gain.Kp[0] = True

            elif header[2] == MC_REG_POSITION_KP_DIV:
                self.position_gain.Kp_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.position_gain.Kp_div[0] = True

            elif header[2] == MC_REG_POSITION_KI:
                self.position_gain.Ki[1] = struct.unpack('<h', message.data[3:5])[0]
                self.position_gain.Ki[0] = True

            elif header[2] == MC_REG_POSITION_KI_DIV:
                self.position_gain.Ki_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.position_gain.Ki_div[0] = True

            elif header[2] == MC_REG_POSITION_KD:
                self.position_gain.Kd[1] = struct.unpack('<h', message.data[3:5])[0]
                self.position_gain.Kd[0] = True

            elif header[2] == MC_REG_POSITION_KD_DIV:
                self.position_gain.Kd_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.position_gain.Kd_div[0] = True
                
            elif header[2] == MC_REG_SPEED_KP:
                self.velocity_gain.Kp[1] = struct.unpack('<h', message.data[3:5])[0]
                self.velocity_gain.Kp[0] = True

            elif header[2] == MC_REG_SPEED_KP_DIV:
                self.velocity_gain.Kp_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.velocity_gain.Kp_div[0] = True

            elif header[2] == MC_REG_SPEED_KI:
                self.velocity_gain.Ki[1] = struct.unpack('<h', message.data[3:5])[0]
                self.velocity_gain.Ki[0] = True

            elif header[2] == MC_REG_SPEED_KI_DIV:
                self.velocity_gain.Ki_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.velocity_gain.Ki_div[0] = True
                
            elif header[2] == MC_REG_I_Q_KP:
                self.iq_gain.Kp[1] = struct.unpack('<h', message.data[3:5])[0]
                self.iq_gain.Kp[0] = True

            elif header[2] == MC_REG_I_Q_KP_DIV:
                self.iq_gain.Kp_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.iq_gain.Kp_div[0] = True

            elif header[2] == MC_REG_I_Q_KI:
                self.iq_gain.Ki[1] = struct.unpack('<h', message.data[3:5])[0]
                self.iq_gain.Ki[0] = True

            elif header[2] == MC_REG_I_Q_KI_DIV:
                self.iq_gain.Ki_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.iq_gain.Ki_div[0] = True
                
            elif header[2] == MC_REG_I_D_KP:
                self.id_gain.Kp[1] = struct.unpack('<h', message.data[3:5])[0]
                self.id_gain.Kp[0] = True

            elif header[2] == MC_REG_I_D_KP_DIV:
                self.id_gain.Kp_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.id_gain.Kp_div[0] = True

            elif header[2] == MC_REG_I_D_KI:
                self.id_gain.Ki[1] = struct.unpack('<h', message.data[3:5])[0]
                self.id_gain.Ki[0] = True

            elif header[2] == MC_REG_I_D_KI_DIV:
                self.id_gain.Ki_div[1] = struct.unpack('<H', message.data[3:5])[0]
                self.id_gain.Ki_div[0] = True


                    
                    

        elif header[0] == Command.START_MOTOR.value:
            if message.data[3] == 1:
                self.power = True
        elif header[0] == Command.STOP_MOTOR.value:
            self.power = False
            
        elif header[0] == Command.USER_GET.value:
            if header[2] == MC_REG_SPEED_MEAS:
                self.velocity = struct.unpack('<f', message.data[3:7])[0]
                
            elif header[2] == MC_REG_I_Q_MEAS:
                self.iq = struct.unpack('<f', message.data[3:7])[0]
                
            elif header[2] == MC_REG_I_D_MEAS:
                self.id = struct.unpack('<f', message.data[3:7])[0]
                
            elif header[2] == U_REG_SPEED_LIMIT:
                print("update_vel_limit")
                self.velocity_limit[0] = struct.unpack('<h', message.data[3:5])[0]
                self.velocity_limit[1] = struct.unpack('<h', message.data[5:7])[0]
                print("vel limit : ", self.velocity_limit)
                
            elif header[2] == U_REG_IQ_LIMIT:
                print("update_iq_limit")
                self.iq_limit[0] = struct.unpack('<h', message.data[3:5])[0]
                self.iq_limit[1] = abs(struct.unpack('<h', message.data[5:7])[0])
                print("iq limit : ", self.iq_limit)
                
            elif header[2] == U_REG_MODE:
                self.mode[1] = message.data[3]
                self.mode[0] = True
        
    def CanDataWaitProcess(self, comm, reg = 0):
        while True:
            message = canable.Read(True)
            if message == None:
                continue
            if message.arbitration_id != self.motorid + 1:
                continue
            self.CanDataProccesser(message)
            header = self.__parse_byte_list(message.data[0:3])
            if header[0] == comm.value and header[2] == reg:
                break
    
    def Update_struct(self, config, data):
        if not config.update:
            size = struct.unpack('<h', data[3:5])[0]
            if size > 3:
                size = 3
            else:
                config.update = True

            config.buffer += (data[5:5+size]   )
            if (len(config.buffer) == config.size):
                return True
        return False
    
    def Update_string(self, config, data):
        if not config[0]:
            size = data[3]
            if size > 4:
                size = 4
            else:
                config[0] = True  
            byte_array = bytes(data[4:4+size])
            config[1] += byte_array.decode('ascii')

                
    def AMPARE2CURRENTUNIT(self, ampare):
        return int(ampare * self.motor_parameter.IQMAX() / self.motor_parameter.Nominal_Current())

    def CURRENTUNIT2AMPARE(self, unit):
        return float(unit  * self.motor_parameter.Nominal_Current()/ self.motor_parameter.IQMAX())
                    
            
            
# if __name__ == "__main__" :

#     canable = CanableClass()
#     motor = Motor(canable, 0x28)
#     canable.SetBaudrate(1000000)
#     canable.Connect(autoRead= False)
#     time.sleep(1)
#     motor.Response()
#     motor.CanDataProccesser(canable.Read(True))
#     motor.Read_Position()
#     motor.CanDataProccesser(canable.Read(True))
    
#     canable.DisConnect()
    
    