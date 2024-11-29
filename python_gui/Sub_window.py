import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt, QPoint, QRect
from PyQt5.QtGui import QPainter, QColor, QFont  
from PyQt5.QtCore import QTimer, pyqtSignal
import numpy as np
import time
from canable import *
from backend import *
from motor_control import *
from PID_gain_profile import *

curr_path = os.path.dirname(os.path.abspath(__file__))
form_class = uic.loadUiType(curr_path+"\Sub_window.ui")[0]

class RotatingBar(QWidget):
    def __init__(self):
        super().__init__()
        self.value = 0
        self.setMinimumSize(200, 200)

    def setValue(self, value):
        self.value = value
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        center = QPoint(self.width() // 2, self.height() // 2)
        length = 100
        angle = self.value
        # QColor(0, 100, 255)
        painter.setPen(QColor(230, 100, 0))
        end_point = QPoint(int(center.x() + length * np.cos(np.radians(angle + 90))),
                           int(center.y() + length * np.sin(np.radians(angle + 90))))
        painter.drawLine(center, end_point)

class SubWindowClass(QDialog, form_class):
    
    closed = pyqtSignal()
    dial_factor = 20
    
    def __init__(self, backend_, id):
        super().__init__()
        self.setupUi(self)
        self.backend = backend_
        self.this_id = id
        self.motor = Motor_Control(self.backend.canable, id)
        self.rotating_bar = RotatingBar()
        self.initdone = False
        
        self.all_info_update_timer = QTimer()
        self.all_info_update_timer.timeout.connect(self.All_info_update)
        self.all_info_update_index = 0
        self.all_info_update_timer.start(10)
        
        self.Mode_torque_button.clicked.connect(self.Motor_Mode_Change)
        self.Mode_velocity_button.clicked.connect(self.Motor_Mode_Change)
        self.Mode_pos_torque_button.clicked.connect(self.Motor_Mode_Change)
        self.Mode_pos_vel_torque_button.clicked.connect(self.Motor_Mode_Change)

        
        self.dial_gridLayout.addWidget(self.rotating_bar, 0, 0)
        self.MotorControl_target_pos_dial.valueChanged.connect(self.Cur_Position_bar_update)
        self.dial_gridLayout.addWidget(self.MotorControl_target_pos_dial, 0, 0)
        self.MotorControl_target_pos_spinbox.valueChanged.connect(self.Cur_Position_spin_update)

        self.MotorControl_motorID.setText(str(hex(id)))
        
        self.Ack_faults_button.clicked.connect(self.Ack_Faults)
        self.motor_start_button.clicked.connect(self.motor.Set_PowerON)
        self.motor_stop_button.clicked.connect(self.motor.Set_PowerOFF)
        
        self.Testbutton.clicked.connect(self.test)
        
        self.Motor_status_update_checkbox.stateChanged.connect(self.motor_status_update_enable)
        self.Gui_update_frq_spinBox.valueChanged.connect(self.Gui_Frequency_Changed)
        self.Gui_update_timer = QTimer()
        self.Gui_update_timer.timeout.connect(self.Gui_update)
        self.Gui_update_timer.start(int(1000/self.Gui_update_frq_spinBox.value()))
        
        
        self.Motor_status_update_frq_spinBox.valueChanged.connect(self.Motor_Frequency_Changed)
        self.Cur_status_request_timer = QTimer()
        self.Cur_status_request_timer.timeout.connect(self.Cur_Status_Request)
        # self.Cur_status_request_timer.start(int(1000/self.Motor_status_update_frq_spinBox.value()))
        
        
        self.Cur_Low_status_request_timer = QTimer()
        self.Cur_Low_status_request_timer.timeout.connect(self.Cur_Low_Status_Request)
        # self.Cur_Low_status_request_timer.start(500)
        
        
        self.MotorControl_target_vel_unit_spinbox.valueChanged.connect(self.Motor_Velocity_Limit_unit_Changed)
        self.MotorControl_target_vel_rpm_spinbox.valueChanged.connect(self.Motor_Velocity_Limit_rpm_Changed)
        self.MotorControl_target_current_unit_spinbox.valueChanged.connect(self.Motor_Current_Limit_unit_Changed)
        self.MotorControl_target_current_A_spinbox.valueChanged.connect(self.Motor_Current_Limit_A_Changed)
        
        self.Mc_config.addItem("More Information")
        self.config_update_button.clicked.connect(self.Config_update)
        
        self.Reset_button.clicked.connect(self.Motor_Reset)
        
        self.position_kp_spinBox.valueChanged.connect(self.Position_Kp_change)
        self.position_kp_div_spinBox.valueChanged.connect(self.Position_Kp_change)
        self.position_ki_spinBox.valueChanged.connect(self.Position_Ki_change)
        self.position_ki_div_spinBox.valueChanged.connect(self.Position_Ki_change)
        self.position_kd_spinBox.valueChanged.connect(self.Position_Kd_change)
        self.position_kd_div_spinBox.valueChanged.connect(self.Position_Kd_change)
        
        self.velocity_kp_spinBox.valueChanged.connect(self.Velocity_Kp_change)
        self.velocity_kp_div_spinBox.valueChanged.connect(self.Velocity_Kp_change)
        self.velocity_ki_spinBox.valueChanged.connect(self.Velocity_Ki_change)
        self.velocity_ki_div_spinBox.valueChanged.connect(self.Velocity_Ki_change)
        
        self.iq_kp_spinBox.valueChanged.connect(self.Iq_Kp_change)
        self.iq_kp_div_spinBox.valueChanged.connect(self.Iq_Kp_change)
        self.iq_ki_spinBox.valueChanged.connect(self.Iq_Ki_change)
        self.iq_ki_div_spinBox.valueChanged.connect(self.Iq_Ki_change)
        
        self.id_kp_spinBox.valueChanged.connect(self.Id_Kp_change)
        self.id_kp_div_spinBox.valueChanged.connect(self.Id_Kp_change)
        self.id_ki_spinBox.valueChanged.connect(self.Id_Ki_change)
        self.id_ki_div_spinBox.valueChanged.connect(self.Id_Ki_change)

        
        
        self.TestLabel.hide()
        self.Testbutton.hide()
        self.label_23.hide()
        self.MotorControl_accelation_spinbox.hide()
    
    def __del__(self):
        self.Cur_status_request_timer.stop()
        self.Gui_update_timer.stop()
        self.Cur_status_request_timer = None
        self.Gui_update_timer = None
        
    def test(self):
        print("Test")
        
        
    def Position_Kp_change(self):
        kp = float(self.position_kp_spinBox.value()) / float(1 << self.position_kp_div_spinBox.value())
        self.position_kp_result_label.setText(f"{kp:.5f}")
        if self.initdone:
            self.motor.Set_Position_Kp(self.position_kp_spinBox.value(), self.position_kp_div_spinBox.value())
        
    def Position_Ki_change(self):
        ki = float(self.position_ki_spinBox.value()) / float(1 << self.position_ki_div_spinBox.value())
        self.position_ki_result_label.setText(f"{ki:.5f}")
        if self.initdone:
            self.motor.Set_Position_Ki(self.position_ki_spinBox.value(), self.position_ki_div_spinBox.value())
        
    def Position_Kd_change(self):
        kd = float(self.position_kd_spinBox.value()) / float(1 << self.position_kd_div_spinBox.value())
        self.position_kd_result_label.setText(f"{kd:.5f}")
        if self.initdone:
            self.motor.Set_Position_Kd(self.position_kd_spinBox.value(), self.position_kd_div_spinBox.value())
    
    def Velocity_Kp_change(self):
        kp = float(self.velocity_kp_spinBox.value()) / float(1 << self.velocity_kp_div_spinBox.value())
        self.velocity_kp_result_label.setText(f"{kp:.5f}")
        if self.initdone:
            self.motor.Set_Velocity_Kp(self.velocity_kp_spinBox.value(), self.velocity_kp_div_spinBox.value())
        
    def Velocity_Ki_change(self):
        ki = float(self.velocity_ki_spinBox.value()) / float(1 << self.velocity_ki_div_spinBox.value())
        self.velocity_ki_result_label.setText(f"{ki:.5f}")
        if self.initdone:
            self.motor.Set_Velocity_Ki(self.velocity_ki_spinBox.value(), self.velocity_ki_div_spinBox.value())
            
    def Iq_Kp_change(self):
        kp = float(self.iq_kp_spinBox.value()) / float(1 << self.iq_kp_div_spinBox.value())
        self.iq_kp_result_label.setText(f"{kp:.5f}")
        if self.initdone:
            self.motor.Set_Iq_Kp(self.iq_kp_spinBox.value(), self.iq_kp_div_spinBox.value())
        
    def Iq_Ki_change(self):
        ki = float(self.iq_ki_spinBox.value()) / float(1 << self.iq_ki_div_spinBox.value())
        self.iq_ki_result_label.setText(f"{ki:.5f}")
        if self.initdone:
            self.motor.Set_Iq_Ki(self.iq_ki_spinBox.value(), self.iq_ki_div_spinBox.value())
            
    def Id_Kp_change(self):
        kp = float(self.id_kp_spinBox.value()) / float(1 << self.id_kp_div_spinBox.value())
        self.id_kp_result_label.setText(f"{kp:.5f}")
        if self.initdone:
            self.motor.Set_Id_Kp(self.id_kp_spinBox.value(), self.id_kp_div_spinBox.value())
        
    def Id_Ki_change(self):
        ki = float(self.id_ki_spinBox.value()) / float(1 << self.id_ki_div_spinBox.value())
        self.id_ki_result_label.setText(f"{ki:.5f}")
        if self.initdone:
            self.motor.Set_Id_Ki(self.id_ki_spinBox.value(), self.id_ki_div_spinBox.value())
        
    def All_info_update(self):
        if self.all_info_update_index == 0:
            self.motor.Read_Stage()
            self.motor.Read_MotorName()
        elif self.all_info_update_index == 1:
            self.Cur_Status_Request()
            self.Cur_Low_Status_Request()
            self.Config_update()
        elif self.all_info_update_index == 2:
            self.motor.Read_Status()
            self.motor.Ack_Faults()
            self.motor.Read_Iq_Limit()
        elif self.all_info_update_index == 3:
            
            self.motor.Read_Velocity_Limit()
            self.motor.Read_USER_Mode()
        elif self.all_info_update_index == 4:
            self.motor.Read_Position_Kp()
            self.motor.Read_Position_Ki()
            self.motor.Read_Position_Kd()
        elif self.all_info_update_index == 5:
            self.motor.Read_Velocity_Kp()
            self.motor.Read_Velocity_Ki()
            self.motor.Read_Iq_Kp()
        elif self.all_info_update_index == 6:
            self.motor.Read_Iq_Ki()
            self.motor.Read_Id_Kp()
            self.motor.Read_Id_Ki()
            self.all_info_update_timer.stop()
            self.all_info_update_index = 0
            
        self.all_info_update_index += 1
        
        
        
    def Config_update(self):
        self.Mc_config.clear()
        self.Mc_config.addItem("More Information")
        self.motor.Read_ApplicationConfig_reg_t()
        self.motor.Read_MotorConfig_reg_t()
        self.motor.Read_FOCFwConfig_reg_t()
        
    def Motor_Mode_Change(self):
        mode = U_Control_Mode.U_POS_VEL_TORQUE_MODE.value
        if self.Mode_torque_button.isChecked():
            mode = U_Control_Mode.U_TORQUE_MODE.value
        elif self.Mode_velocity_button.isChecked():
            mode = U_Control_Mode.U_VELOCITY_MODE.value
        elif self.Mode_pos_torque_button.isChecked():
            mode = U_Control_Mode.U_POSITION_TORQUE_MODE.value
        elif self.Mode_pos_vel_torque_button.isChecked():
            mode = U_Control_Mode.U_POS_VEL_TORQUE_MODE.value
        self.motor.Stop_motor()
        self.motor.Set_USER_Mode(mode)
        
    def Motor_Reset(self):
        self.motor.Reset()
    
    def Motor_Current_Limit_unit_Changed(self, value):
        if self.initdone:
            print("Motor_Current_Limit_unit_Changed ", value, " ", self.motor.CURRENTUNIT2AMPARE(value))
            if self.motor.iq_limit[0]==0xFFFFFFFF:
                self.motor.Read_Iq_Limit()
            else:
                self.motor.Set_Iq_Limit(-value, value)
                print(value)
            # self.MotorControl_target_current_A_spinbox.setValue(self.motor.CURRENTUNIT2AMPARE(value))
                self.motor.Read_Iq_Limit()
    
    def Motor_Current_Limit_A_Changed(self, value):
        if self.initdone:
            print("Motor_Current_Limit_A_Changed", value, " ", self.motor.AMPARE2CURRENTUNIT(value))
            if self.motor.iq_limit[0] == 0xFFFFFFFF:
                self.motor.Read_Iq_Limit()
            else:
                self.motor.Set_Iq_Limit(-self.motor.AMPARE2CURRENTUNIT(value), self.motor.AMPARE2CURRENTUNIT(value))
            # self.MotorControl_target_current_unit_spinbox.setValue(self.motor.AMPARE2CURRENTUNIT(value))
                self.motor.Read_Iq_Limit()
            
    def Motor_Velocity_Limit_unit_Changed(self, value):
        if self.initdone:
            if self.motor.velocity_limit[0] == 0xFFFFFFFF:
                self.motor.Read_Velocity_Limit()
            else:
                self.motor.Set_Velocity_Limit(-value, value)
            value = self.motor.motor_parameter.SPEEDUNIT2RPM(value)
            # self.MotorControl_target_vel_rpm_spinbox.setValue(float(value))
            self.motor.Read_Velocity_Limit()
    
    def Motor_Velocity_Limit_rpm_Changed(self, value):
        if self.initdone:
            value = self.motor.motor_parameter.RPM2SPEEDUNIT(value)
            if self.motor.velocity_limit == 0xFFFFFFFF:
                self.motor.Read_Velocity_Limit()
            else:
                self.motor.Set_Velocity_Limit(-value, value)
            # self.MotorControl_target_vel_unit_spinbox.setValue(value)
            self.motor.Read_Velocity_Limit()
        
    def Gui_Frequency_Changed(self, value):
        self.Gui_update_timer.stop()
        self.Gui_update_timer.start(int(1000/value))
    
    def Motor_Frequency_Changed(self, value):
        self.Cur_status_request_timer.stop()
        self.Cur_status_request_timer.start(int(1000/value))
      
    def motor_status_update_enable(self, state):
        if state == 0:
            print("Motor Status Update: Off")
            self.Cur_status_request_timer.stop()
            self.Cur_Low_status_request_timer.stop()
        elif state == 2:
            print("Motor Status Update: On")
            self.Cur_status_request_timer.start(int(1000 / self.Motor_status_update_frq_spinBox.value()))
            self.Cur_Low_status_request_timer.start(500)
            
    def Ack_Faults(self):
        self.motor.Ack_Faults()
        
    def Cur_Status_Request(self):
        self.motor.Read_Status()
        self.motor.Read_Faults()
        self.motor.Read_Position()
        self.motor.Read_Velocity_F()
        self.motor.Read_Iq_F()
        self.motor.Read_Id_F()
        self.motor.Read_Encoder_Speed()
        
    def Cur_Low_Status_Request(self):
        self.motor.Read_Power()
        self.motor.Read_Temperature()
        self.motor.Read_Vbus()
    
    def Motor_mode_ChangeDial(self, mode):
        if mode == U_Control_Mode.U_TORQUE_MODE.value:
            self.MotorControl_target_pos_dial.setWrapping(False)
            self.MotorControl_target_pos_dial.setRange(-int(self.motor.motor_parameter.IQMAX()), int(self.motor.motor_parameter.IQMAX()))
            self.MotorControl_target_pos_spinbox.setRange(-int(self.motor.motor_parameter.IQMAX()), int(self.motor.motor_parameter.IQMAX()))
            self.MotorControl_target_pos_dial.setValue(0)
            self.MotorControl_target_vel_unit_spinbox.setEnabled(False)
            self.MotorControl_target_vel_rpm_spinbox.setEnabled(False)
            self.MotorControl_target_current_unit_spinbox.setEnabled(False)
            self.MotorControl_target_current_A_spinbox.setEnabled(False)
            self.dial_factor = 1
        elif mode == U_Control_Mode.U_VELOCITY_MODE.value:
            self.MotorControl_target_pos_dial.setWrapping(False)
            self.MotorControl_target_pos_dial.setRange(-self.motor.motor_parameter.MAX_APPLICATION_SPEED_UNIT()
                                                       , self.motor.motor_parameter.MAX_APPLICATION_SPEED_UNIT())
            self.MotorControl_target_pos_spinbox.setRange(-self.motor.motor_parameter.MAX_APPLICATION_SPEED_UNIT()
                                                          , self.motor.motor_parameter.MAX_APPLICATION_SPEED_UNIT())
            self.MotorControl_target_pos_dial.setValue(0)
            self.MotorControl_target_vel_unit_spinbox.setEnabled(False)
            self.MotorControl_target_vel_rpm_spinbox.setEnabled(False)
            self.MotorControl_target_current_unit_spinbox.setEnabled(True)
            self.MotorControl_target_current_A_spinbox.setEnabled(True)
            self.dial_factor = 1
        elif mode == U_Control_Mode.U_POSITION_TORQUE_MODE.value:
            self.MotorControl_target_pos_dial.setWrapping(True)
            self.MotorControl_target_pos_dial.setRange(-3600, 3600)
            self.MotorControl_target_pos_spinbox.setRange(-INT32_MAX, INT32_MAX)
            self.MotorControl_target_pos_dial.setValue(0)
            self.MotorControl_target_vel_unit_spinbox.setEnabled(False)
            self.MotorControl_target_vel_rpm_spinbox.setEnabled(False)
            self.MotorControl_target_current_unit_spinbox.setEnabled(True)
            self.MotorControl_target_current_A_spinbox.setEnabled(True)
            self.dial_factor = 20
        elif mode == U_Control_Mode.U_POS_VEL_TORQUE_MODE.value:
            self.MotorControl_target_pos_dial.setWrapping(True)
            self.MotorControl_target_pos_dial.setRange(-3600, 3600)
            self.MotorControl_target_pos_spinbox.setRange(-INT32_MAX, INT32_MAX)
            self.MotorControl_target_pos_dial.setValue(0)
            self.MotorControl_target_vel_unit_spinbox.setEnabled(True)
            self.MotorControl_target_vel_rpm_spinbox.setEnabled(True)
            self.MotorControl_target_current_unit_spinbox.setEnabled(True)
            self.MotorControl_target_current_A_spinbox.setEnabled(True)
            self.dial_factor = 20
    
    def Cur_Position_bar_update(self, value):
        if self.motor.mode[1] == U_Control_Mode.U_POSITION_TORQUE_MODE.value or self.motor.mode == U_Control_Mode.U_POSITION_TORQUE_MODE.value:
            self.motor.Set_Position((float(value) / self.dial_factor) / 180 *np.pi)
        elif self.motor.mode[1] == U_Control_Mode.U_VELOCITY_MODE.value:
            self.motor.Ramp_Velocity(speed = value)
        elif self.motor.mode[1] == U_Control_Mode.U_TORQUE_MODE.value:
            self.motor.Ramp_Torque(torque = value)
        self.MotorControl_target_pos_spinbox.setValue(float(value) / self.dial_factor)

    def Cur_Position_spin_update(self, value):
        self.motor.Set_Position(value / 180 * np.pi)
        self.MotorControl_target_pos_dial.setValue(int(value) * self.dial_factor)
        
    def Gui_update(self):
        if self.motor.reset_signal:
            self.motor.reset_signal = False
            self.all_info_update_timer.start(10)
        
        self.Motor_param_update(self.motor.motor_parameter)
        if not self.initdone:
            return
        self.Motor_Status_Writer(self.motor.status)
        self.Motor_faults_update(self.motor.faults)
        self.Motor_position_update(self.motor.position)
        self.Motor_velocity_update(self.motor.velocity)
        self.Motor_Iqd_update(self.motor.iq, self.motor.id)
        self.Motor_low_status_update(self.motor.vbus, self.motor.temperature, self.motor.power)
        self.Motor_Limit_update(self.motor.position_limit[1],
                                self.motor.velocity_limit[1], 
                                self.motor.iq_limit[1])
        if self.motor.mode[0]:
            self.Motor_Mode_Changed(self.motor.mode[1])
            self.Motor_mode_ChangeDial(self.motor.mode[1])
        self.Position_Gain_update(self.motor)
        self.Velocity_Gain_update(self.motor)
        self.I_Gain_update(self.motor)
        
        
    def Motor_Status_Writer(self, state):
        font = QFont("Bahnschrift", 16, QFont.Bold)
        self.Cur_State.setFont(font)
        if state == MC_State.ICLWAIT.value:
            self.Cur_State.setStyleSheet("color: #00aa00;")
            self.Cur_State.setText("ICLWAIT")
        elif state == MC_State.IDLE.value:
            self.Cur_State.setStyleSheet("color: #00aa00;")
            self.Cur_State.setText("IDLE")
        elif state == MC_State.ALIGNMENT.value:
            self.Cur_State.setStyleSheet("color: #00aa00;")
            self.Cur_State.setText("ALIGNMENT")
        elif state == MC_State.CHARGE_BOOT_CAP.value:
            self.Cur_State.setStyleSheet("color: #00aa00;")
            self.Cur_State.setText("CHARGE_BOOT_CAP")
        elif state == MC_State.OFFSET_CALIB.value:
            self.Cur_State.setStyleSheet("color: #0000ee;")
            self.Cur_State.setText("OFFSET_CALIB")
        elif state == MC_State.START.value:
            self.Cur_State.setStyleSheet("color: #0000ee;")
            self.Cur_State.setText("START")
        elif state == MC_State.SWITCH_OVER.value:
            self.Cur_State.setStyleSheet("color: #ee0000;")
            self.Cur_State.setText("SWITCH_OVER")
        elif state == MC_State.RUN.value:
            self.Cur_State.setStyleSheet("color: #0000ee;")
            self.Cur_State.setText("RUN")
        elif state == MC_State.STOP.value:
            self.Cur_State.setStyleSheet("color: #0000ee;")
            self.Cur_State.setText("STOP")
        elif state == MC_State.FAULT_NOW.value:
            self.Cur_State.setStyleSheet("color: #ee0000;")
            self.Cur_State.setText("FAULT_NOW")
        elif state == MC_State.FAULT_OVER.value:
            self.Cur_State.setStyleSheet("color: #ee0000;")
            self.Cur_State.setText("FAULT_OVER")
        elif state == MC_State.WAIT_STOP_MOTOR.value:
            self.Cur_State.setStyleSheet("color: #0000ee;")
            self.Cur_State.setText("WAIT_STOP_MOTOR")
    
    def Motor_faults_update(self, fault):
        font = QFont("Bahnschrift SemiBold", 12, QFont.Bold)  # 폰트 이름, 크기, 스타일 설정
        self.Faults_state.setFont(font)
        if fault == MC_NO_ERROR:
            self.Faults_state.setStyleSheet("color: #000000;")
            self.Faults_state.setText("No Error")
        elif fault == MC_DURATION:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Foc Duration")
        elif fault == MC_OVER_VOLT:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Over Voltage")
        elif fault == MC_UNDER_VOLT:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Under Voltage")
        elif fault == MC_OVER_TEMP:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Over Heat")
        elif fault == MC_START_UP:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Start Up Failure")
        elif fault == MC_SPEED_FDBK:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Speed Feedback")
        elif fault == MC_OVER_CURR:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Over Current")
        elif fault == MC_SW_ERROR:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Software Error")
        elif fault == MC_DP_FAULT:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Driver Protection")
        elif fault == MC_OVER_RUN:
            self.Faults_state.setStyleSheet("color: #ee0000;")
            self.Faults_state.setText("Over Run")
            
    def Motor_position_update(self, position):
        self.Cur_position_radian.setText(f"{position:.5f}")
        degree = position * 180 / np.pi
        if self.motor.mode[1] == U_Control_Mode.U_POS_VEL_TORQUE_MODE.value or self.motor.mode[1] == U_Control_Mode.U_POSITION_TORQUE_MODE.value:
            self.rotating_bar.setValue(degree-180)
        font = QFont("Bahnschrift", 15, QFont.Bold)
        self.Cur_postion_degree_dial.setFont(font)
        self.Cur_postion_degree_dial.setText(f"{degree:.1f}")
        self.Cur_postion_degree.setText(f"{degree:.5f}")
        
    def Motor_velocity_update(self, velocity):
        self.Cur_velocity_rpm.setText(f"{velocity:.1f}")
        self.Cur_velocity_radianpersec.setText(f"{velocity * 60 * 2 * np.pi:.2f}")
        if self.motor.mode[1] == U_Control_Mode.U_VELOCITY_MODE.value:
            self.rotating_bar.setValue(self.motor.motor_parameter.RPM2SPEEDUNIT(velocity)-180)
        self.TestLabel.setText(str(self.motor.encoder_speed))
        
    def Motor_Iqd_update(self, iq, id):
        iq = float(iq)
        id = float(id)
        if self.motor.mode[1] == U_Control_Mode.U_TORQUE_MODE.value:
            self.rotating_bar.setValue((iq * 720 / self.motor.motor_parameter.Max_Readable_Current()) * 350/360 - 180)
            # self.rotating_bar.setValue(iq-180)
        self.Cur_Iq.setText(f"{iq:.3f}")
        self.Cur_Id.setText(f"{id:.3f}")
    
    def Motor_Limit_update(self, position, velocity, current):
        a= 0
        # print("Motor_Limit_update ", velocity, " ", current)
        if velocity != 0xFFFFFFFF:
            self.MotorControl_target_vel_unit_spinbox.setValue(int(velocity))
        if current != 0xFFFFFFFF:
            self.MotorControl_target_current_unit_spinbox.setValue(int(current))
        
    def Motor_low_status_update(self, vbus, temp, power):
        self.temperature_data.setText(str(temp))
        self.power_data.setText(f"{power:.3f}")
        self.vbus_data.setText(str(vbus))
        
    def Motor_Mode_Changed(self, mode):
        if mode == U_Control_Mode.U_TORQUE_MODE.value:
            self.Mode_torque_button.setChecked(True)
        elif mode == U_Control_Mode.U_VELOCITY_MODE.value:
            self.Mode_velocity_button.setChecked(True)
        elif  mode == U_Control_Mode.U_POSITION_TORQUE_MODE.value:
            self.Mode_pos_torque_button.setChecked(True)
        elif mode == U_Control_Mode.U_POS_VEL_TORQUE_MODE.value:
            self.Mode_pos_vel_torque_button.setChecked(True)
        self.motor.mode[0] = False
        
    def Motor_param_update(self, motor_param:Motor_Param):
        if motor_param.power_stage[0]:
            self.power_stage_label.setText(motor_param.power_stage[1])
            motor_param.power_stage[0] = False
        if motor_param.motor_name[0]:
            self.motor_name_label.setText(motor_param.motor_name[1])
            motor_param.motor_name[0] = False
        if motor_param.application_config.update:
            self.max_speed_label.setText(str(motor_param.Max_Motor_Speed()))
            self.max_readable_current_label.setText(f"{motor_param.Max_Readable_Current():.3f}")
            self.nominal_current_label.setText(f"{motor_param.Nominal_Current():.3f}")
            self.nominal_voltage_label.setText(str(motor_param.Nominal_Voltage()))
            self.contorl_type_label.setText(motor_param.Control_Type())
            motor_param.application_config.update = False
            data = [value for value in motor_param.application_config.generate_application_config_string().split('\n')]
            self.Mc_config.addItems(data)
        if motor_param.motor_config.update:
            motor_param.motor_config.update = False
            data = [value for value in motor_param.motor_config.generate_motor_config_string().split('\n')]
            self.Mc_config.addItems(data)
        if motor_param.focfw_config.update:    
            self.primary_speed_sensor_label.setText(motor_param.Primary_Speed_sensor())
            self.aux_speed_sensor_label.setText(motor_param.Aux_Speed_Sensor())
            self.current_sensing_topology_label.setText(motor_param.Current_Sensing_Topology())
            self.foc_rate_label.setText(str(motor_param.FOC_rate()))
            self.pwn_frequency_label.setText(str(motor_param.PWM_Frequency()))
            self.medium_frequency_label.setText(str(motor_param.Medium_Frequency()))
            self.feed_forward_label.setText(motor_param.Feed_Forward("status"))
            self.position_control_label.setText(motor_param.Position_Control("status"))
            self.vbus_sensing_label.setText(motor_param.VBus_Sensing("status"))
            self.temperature_sensing_label.setText(motor_param.Temperature_Sensing("status"))
            motor_param.focfw_config.update = False
            data = [value for value in motor_param.focfw_config.generate_focfw_config_string().split('\n')]
            self.Mc_config.addItems(data)
            self.MotorControl_target_vel_unit_spinbox.setRange(1, self.motor.motor_parameter.MAX_APPLICATION_SPEED_UNIT())
            self.MotorControl_target_vel_rpm_spinbox.setRange(1, self.motor.motor_parameter.Max_Motor_Speed())
            self.MotorControl_target_current_unit_spinbox.setRange(1, int(self.motor.motor_parameter.IQMAX()))
            self.MotorControl_target_current_A_spinbox.setRange(0.01, int(self.motor.motor_parameter.Nominal_Current()))
            
            self.initdone = True

    def Position_Gain_update(self, motor:Motor_Control):
        change = False
        if motor.position_gain.Kp[0]:
            self.position_kp_spinBox.setValue(motor.position_gain.Kp[1])
            motor.position_gain.Kp[0] = False
            change = True
        if motor.position_gain.Kp_div[0]:
            self.position_kp_div_spinBox.setValue(motor.position_gain.Kp_div[1])
            motor.position_gain.Kp_div[0] = False
            change = True
        if change:
            self.Position_Kp_change()
        change = False
        if motor.position_gain.Ki[0]:
            self.position_ki_spinBox.setValue(motor.position_gain.Ki[1])
            motor.position_gain.Ki[0] = False
            change = True
        if motor.position_gain.Ki_div[0]:
            self.position_ki_div_spinBox.setValue(motor.position_gain.Ki_div[1])
            motor.position_gain.Ki_div[0] = False
            change = True
        if change:
            self.Position_Ki_change()
        change = False
        if motor.position_gain.Kd[0]:
            self.position_kd_spinBox.setValue(motor.position_gain.Kd[1])
            motor.position_gain.Kd[0] = False
            change = True
        if motor.position_gain.Kd_div[0]:
            self.position_kd_div_spinBox.setValue(motor.position_gain.Kd_div[1])
            motor.position_gain.Kd_div[0] = False
            change = True
        if change:
            self.Position_Kd_change()
            
    def Velocity_Gain_update(self, motor:Motor_Control):
        change = False
        if motor.velocity_gain.Kp[0]:
            self.velocity_kp_spinBox.setValue(motor.velocity_gain.Kp[1])
            motor.velocity_gain.Kp[0] = False
            change = True
        if motor.velocity_gain.Kp_div[0]:
            self.velocity_kp_div_spinBox.setValue(motor.velocity_gain.Kp_div[1])
            motor.velocity_gain.Kp_div[0] = False
            change = True
        if change:
            self.Velocity_Kp_change()
        change = False
        if motor.velocity_gain.Ki[0]:
            self.velocity_ki_spinBox.setValue(motor.velocity_gain.Ki[1])
            motor.velocity_gain.Ki[0] = False
            change = True
        if motor.velocity_gain.Ki_div[0]:
            self.velocity_ki_div_spinBox.setValue(motor.velocity_gain.Ki_div[1])
            motor.velocity_gain.Ki_div[0] = False
            change = True
        if change:
            self.Velocity_Ki_change()
        
            
    def I_Gain_update(self, motor:Motor_Control):
        change = False
        if motor.iq_gain.Kp[0]:
            self.iq_kp_spinBox.setValue(motor.iq_gain.Kp[1])
            motor.iq_gain.Kp[0] = False
            change = True
        if motor.iq_gain.Kp_div[0]:
            self.iq_kp_div_spinBox.setValue(motor.iq_gain.Kp_div[1])
            motor.iq_gain.Kp_div[0] = False
            change = True
        if change:
            self.Iq_Kp_change()
        change = False
        if motor.iq_gain.Ki[0]:
            self.iq_ki_spinBox.setValue(motor.iq_gain.Ki[1])
            motor.iq_gain.Ki[0] = False
            change = True
        if motor.iq_gain.Ki_div[0]:
            self.iq_ki_div_spinBox.setValue(motor.iq_gain.Ki_div[1])
            motor.iq_gain.Ki_div[0] = False
            change = True
        if change:
            self.Iq_Ki_change()
        change = False
        if motor.id_gain.Kp[0]:
            self.id_kp_spinBox.setValue(motor.id_gain.Kp[1])
            motor.id_gain.Kp[0] = False
            change = True
        if motor.id_gain.Kp_div[0]:
            self.id_kp_div_spinBox.setValue(motor.id_gain.Kp_div[1])
            motor.id_gain.Kp_div[0] = False
            change = True
        if change:
            self.Id_Kp_change()
        change = False
        if motor.id_gain.Ki[0]:
            self.id_ki_spinBox.setValue(motor.id_gain.Ki[1])
            motor.id_gain.Ki[0] = False
            change = True
        if motor.id_gain.Ki_div[0]:
            self.id_ki_div_spinBox.setValue(motor.id_gain.Ki_div[1])
            motor.id_gain.Ki_div[0] = False
            change = True
        if change:
            self.Id_Ki_change()


            
            
    def CAN_data_Recive(self, message):
        self.motor.CanDataProccesser(message)
        
    def closeEvent(self, event):
        self.closed.emit()
        event.accept()
        self.__del__()
        
if __name__ == "__main__" :
    
    canable = CanableClass()
    backend = BackEnd(dev = canable)
    
    device_list = backend.SearchDevice()
    if device_list == None or len(device_list) == 0:
        print("Can't find Any Device")
        exit()
    backend.SetDeviceBaudrate(1000000)

    if not backend.DevConnect():
        print("Can't Connect Device")
        exit()
    app = QApplication(sys.argv)

    backend.start_motor(0x28)


    app.exec_()
    backend.DevDisConnect()
    