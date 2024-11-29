from enum import Enum, auto
import struct

MCP_ID_SIZE = 2  # Number of bytes
MCP_ID_SIZE_16B = 1  # Number of 16-bit words
ELT_IDENTIFIER_POS = 6
TYPE_POS = 3
TYPE_MASK = 0x38  # 0b00111000
MOTOR_MASK = 0x7  # 0b00000111
REG_MASK = 0xFFF8  # 0b1111111111111000

"""Extract motor ID from the given data ID."""
def extract_motor_id(data_id): 
    return (data_id - 1) & MOTOR_MASK

# Type definitions with bit shifts
TYPE_DATA_SEG_END = (0 << TYPE_POS)
TYPE_DATA_8BIT = (1 << TYPE_POS)
TYPE_DATA_16BIT = (2 << TYPE_POS)
TYPE_DATA_32BIT = (3 << TYPE_POS)
TYPE_DATA_STRING = (4 << TYPE_POS)
TYPE_DATA_RAW = (5 << TYPE_POS)
TYPE_DATA_FLAG = (6 << TYPE_POS)
TYPE_DATA_SEG_BEG = (7 << TYPE_POS)

U_TYPE_DATA_SEG_END = 1
U_TYPE_DATA_8BIT = 2
U_TYPE_DATA_16BIT = 3
U_TYPE_DATA_32BIT = 4
U_TYPE_DATA_STRING = 5
U_TYPE_DATA_RAW = 6
U_TYPE_DATA_5BYTE = 7


# Constants
ELT_IDENTIFIER_POS = 6

# TYPE_DATA_8BIT registers definition
MC_REG_STATUS = (1 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_CONTROL_MODE = (2 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_RUC_STAGE_NBR = (3 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_PFC_STATUS = (13 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_PFC_ENABLED = (14 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_SC_CHECK = (15 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_SC_STATE = (16 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_SC_STEPS = (17 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_SC_PP = (18 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_SC_FOC_REP_RATE = (19 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_SC_COMPLETED = (20 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_POSITION_CTRL_STATE = (21 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_POSITION_ALIGN_STATE = (22 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_HT_STATE = (23 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_HT_PROGRESS = (24 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_HT_PLACEMENT = (25 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_HT_MECH_WANTED_DIRECTION = (26 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_LOWSIDE_MODULATION = (27 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_QUASI_SYNCH = (28 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_PB_CHARACTERIZATION = (29 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_OPENLOOP_DC = (30 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_COMMUTATION_STEPBUFSIZE = (31 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_OPENLOOP = (32 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_OPENLOOP_REVUP = (33 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_OPENLOOP_VOLTFACTOR = (34 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT
MC_REG_OPENLOOP_SENSING = (35 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT

U_REG_MODE = (36 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT

# TYPE_DATA_16BIT registers definition
MC_REG_SPEED_KP = (2 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_SPEED_KI = (3 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_SPEED_KD = (4 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_KP = (6 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_KI = (7 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_KD = (8 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_KP = (10 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_KI = (11 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_KD = (12 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_C1 = (13 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_C2 = (14 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_C1 = (15 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_C2 = (16 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_KI = (17 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_KP = (18 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FLUXWK_KP = (19 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FLUXWK_KI = (20 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FLUXWK_BUS = (21 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_BUS_VOLTAGE = (22 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_HEATS_TEMP = (23 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_DAC_OUT1 = (25 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_DAC_OUT2 = (26 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_DAC_OUT3 = (27 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FLUXWK_BUS_MEAS = (30 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_A = (31 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_B = (32 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_ALPHA_MEAS = (33 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_BETA_MEAS = (34 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_MEAS = (35 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_MEAS = (36 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_REF = (37 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_REF = (38 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_V_Q = (39 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_V_D = (40 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_V_ALPHA = (41 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_V_BETA = (42 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_ENCODER_EL_ANGLE = (43 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_ENCODER_SPEED = (44 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_EL_ANGLE = (45 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_ROT_SPEED = (46 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_I_ALPHA = (47 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_I_BETA = (48 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_BEMF_ALPHA = (49 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_BEMF_BETA = (50 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_EL_ANGLE = (51 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_ROT_SPEED = (52 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_I_ALPHA = (53 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_I_BETA = (54 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_BEMF_ALPHA = (55 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOCORDIC_BEMF_BETA = (56 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_DAC_USER1 = (57 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_DAC_USER2 = (58 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_HALL_EL_ANGLE = (59 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_HALL_SPEED               = (60 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FF_VQ                    = (62 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FF_VD                    = (63 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FF_VQ_PIOUT              = (64 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FF_VD_PIOUT              = (65 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_DCBUS_REF            = (66 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_DCBUS_MEAS           = (67 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_ACBUS_FREQ           = (68 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_ACBUS_RMS            = (69 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_I_KP                 = (70 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_I_KI                 = (71 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_I_KD                 = (72 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_V_KP                 = (73 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_V_KI                 = (74 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_V_KD                 = (75 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_STARTUP_DURATION     = (76 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_SC_PWM_FREQUENCY         = (77 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_POSITION_KP              = (78 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_POSITION_KI              = (79 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_POSITION_KD              = (80 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_SPEED_KP_DIV             = (81 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_SPEED_KI_DIV             = (82 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_SPEED_KD_DIV             = (83 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_KP_DIV               = (84 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_KI_DIV               = (85 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_D_KD_DIV               = (86 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_KP_DIV               = (87 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_KI_DIV               = (88 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_I_Q_KD_DIV               = (89 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_POSITION_KP_DIV          = (90 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_POSITION_KI_DIV          = (91 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_POSITION_KD_DIV          = (92 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_I_KP_DIV             = (93 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_I_KI_DIV             = (94 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_I_KD_DIV             = (95 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_V_KP_DIV             = (96 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_V_KI_DIV             = (97 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PFC_V_KD_DIV             = (98 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_KI_DIV            = (99 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STOPLL_KP_DIV            = (100 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FLUXWK_KP_DIV            = (101 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FLUXWK_KI_DIV            = (102 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_STARTUP_CURRENT_REF      = (105 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_PULSE_VALUE              = (106 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_FOC_VQREF                = (107 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_OVERVOLTAGETHRESHOLD     = (112 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_UNDERVOLTAGETHRESHOLD    = (113 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT
MC_REG_OPENLOOP_CURRFACTOR      = (108 << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT

U_REG_POSITION_UPPER_LIMIT      = (117 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
U_REG_POSITION_LOWER_LIMIT      = (118 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
U_REG_SPEED_LIMIT               = (119 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
U_REG_IQ_LIMIT                  = (120 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT


# TYPE_DATA_32BIT registers definition
MC_REG_FAULTS_FLAGS = (0 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SPEED_MEAS = (1 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SPEED_REF = (2 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_STOPLL_EST_BEMF = (3 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT  # To check shifted by >> 16
MC_REG_STOPLL_OBS_BEMF = (4 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT  # To check shifted by >> 16
MC_REG_STOCORDIC_EST_BEMF = (5 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT  # To check shifted by >> 16
MC_REG_STOCORDIC_OBS_BEMF = (6 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT  # To check shifted by >> 16
MC_REG_FF_1Q = (7 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT  # To check shifted by >> 16
MC_REG_FF_1D = (8 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT  # To check shifted by >> 16
MC_REG_FF_2 = (9 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT  # To check shifted by >> 16
MC_REG_PFC_FAULTS = (40 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_CURRENT_POSITION = (41 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_RS = (91 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_LS = (92 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_KE = (93 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_VBUS = (94 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_MEAS_NOMINALSPEED = (95 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_CURRENT = (96 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_SPDBANDWIDTH = (97 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_LDLQRATIO = (98 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_NOMINAL_SPEED = (99 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_CURRBANDWIDTH = (100 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_J = (101 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_F = (102 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_MAX_CURRENT = (103 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_STARTUP_SPEED = (104 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_SC_STARTUP_ACC = (105 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT
MC_REG_RESISTOR_OFFSET = (116 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT

MC_REG_MOTOR_POWER = (109 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT

# TYPE_DATA_STRING registers definition
MC_REG_FW_NAME = (0 << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING
MC_REG_CTRL_STAGE_NAME = (1 << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING
MC_REG_PWR_STAGE_NAME = (2 << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING
MC_REG_MOTOR_NAME = (3 << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING

# TYPE_DATA_RAW registers definition
MC_REG_GLOBAL_CONFIG = (0 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_MOTOR_CONFIG = (1 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_APPLICATION_CONFIG = (2 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_FOCFW_CONFIG = (3 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_SCALE_CONFIG = (4 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_SPEED_RAMP = (6 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_TORQUE_RAMP = (7 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_REVUP_DATA = (8 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW  # Configure all steps
MC_REG_CURRENT_REF = (13 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_POSITION_RAMP = (14 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_ASYNC_UARTA = (20 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_ASYNC_UARTB = (21 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_ASYNC_STLNK = (22 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_HT_HEW_PINS = (28 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_HT_CONNECTED_PINS = (29 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_HT_PHASE_SHIFT = (30 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
MC_REG_BEMF_ADC_CONF = (31 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW
print(MC_REG_CURRENT_REF)

MC_NO_ERROR    = 0x0000 # /**< @brief No error. */
MC_NO_FAULTS   = 0x0000 #/**< @brief No error. */
MC_DURATION    = 0x0001 # /**< @brief Error: FOC rate to high. */
MC_OVER_VOLT   = 0x0002 # /**< @brief Error: Software over voltage. */
MC_UNDER_VOLT  = 0x0004 # /**< @brief Error: Software under voltage. */
MC_OVER_TEMP   = 0x0008 # /**< @brief Error: Software over temperature. */
MC_START_UP    = 0x0010 # /**< @brief Error: Startup failed. */
MC_SPEED_FDBK  = 0x0020 # /**< @brief Error: Speed feedback. */
MC_OVER_CURR   = 0x0040 # /**< @brief Error: Emergency input (Over current). */
MC_SW_ERROR    = 0x0080 # /**< @brief Software Error. */
MC_DP_FAULT    = 0x0400 # /**< @brief Error Driver protection fault. */
MC_OVER_RUN    = 0x0800

RESETSIGNAL = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
print(RESETSIGNAL)


# configurationFlag1 definition 
FLUX_WEAKENING_FLAG =      (1)
FEED_FORWARD_FLAG =        (1 << 1)
MTPA_FLAG =                (1 << 2)
PFC_FLAG =                 (1 << 3)
ICL_FLAG =                  (1 << 4)
RESISTIVE_BREAK_FLAG =      (1 << 5)
OCP_DISABLE_FLAG =          (1 << 6)
STGAP_FLAG =                (1 << 7)
POSITION_CTRL_FLAG =        (1 << 8)
VBUS_SENSING_FLAG =         (1 << 9)
TEMP_SENSING_FLAG =         (1 << 10)
VOLTAGE_SENSING_FLAG =      (1 << 11)
FLASH_CONFIG_FLAG =         (1 << 12)
DAC_CH1_FLAG =              (1 << 13)
DAC_CH2_FLAG =              (1 << 14)
OTF_STARTUP_FLAG =          (1 << 15)

# configurationFlag2 definition 
OVERMODULATION_FLAG =       (1)
DISCONTINUOUS_PWM_FLAG =    (1 << 1)
PROFILER_FLAG =             (1 << 13)
DBG_MCU_LOAD_MEASURE_FLAG = (1 << 14)
DBG_OPEN_LOOP_FLAG =        (1 << 15)




class Speed_Sensor(Enum):
    ENO_SENSOR =                0
    EPLL =                      1
    ECORDIC =                   2
    EENCODER =                  3
    EHALL =                     4
    EHSO =                      5
    EZEST =                     6

class MC_State(Enum):
    ICLWAIT = 12
    IDLE = 0
    ALIGNMENT = 2
    CHARGE_BOOT_CAP = 16
    OFFSET_CALIB = 17
    START = 4
    SWITCH_OVER = 19
    RUN = 6
    STOP = 8
    FAULT_NOW = 10
    FAULT_OVER = 11
    WAIT_STOP_MOTOR = 20
    
class MC_Control_Mode(Enum):
    MCM_OBSERVING_MODE = 0
    MCM_OPEN_LOOP_VOLTAGE_MODE = 1
    MCM_OPEN_LOOP_CURRENT_MODE = 2
    MCM_SPEED_MODE = 3
    MCM_TORQUE_MODE = 4
    MCM_PROFILING_MODE = 5
    MCM_SHORTED_MODE = 6
    MCM_POSITION_MODE = 7
    MCM_MODE_NUM = 8
    
class U_Control_Mode(Enum):
    U_TORQUE_MODE = 0
    U_VELOCITY_MODE = 1
    U_POSITION_TORQUE_MODE = 2
    U_POS_VEL_TORQUE_MODE = 3
    
class Command(Enum):
    RESPONSE = 0x00
    SET_DATA_ELEMENT = 0x01
    GET_DATA_ELEMENT = 0x02
    START_MOTOR = 0x03
    STOP_MOTOR = 0x04
    USER_SET = 0x05
    USER_GET = 0x06
    FALULT_ACK = 0x07

U_STOP_RAMP = 0x01
U_POSITION = 0x02 # 데이터와 동시에 시작
U_VELOCITY = 0x03 
U_IQ = 0x04
U_ID = 0x05
U_RESET = 0xFF

INT32_MAX = 0x7FFFFFFF
INT16_MAX = 0x7FFF
INT8_MAX = 0x7F
UINT32_MAX = 0xFFFFFFFF
UINT16_MAX = 0xFFFF
UINT8_MAX = 0xFF

print(hex(MC_REG_FOCFW_CONFIG))

class Aplication_Config:
    def __init__(self):
        self.update = False
        self.buffer = list()
        self.size = 16
        self.maxMechanicalSpeed = 1     # uint32_t
        self.maxReadableCurrent = 1     # float_t 
        self.nominalCurrent=1           # float_t
        self.nominalVoltage=1           # uint16_t
        self.driveType=0                # uint8_t
        self.padding=0                  # uint8_t
        
    def DriveType(self):
        if self.driveType == 0:
            return "FOC"
        elif self.driveType == 1:
            return "Six-Step"
        
    def Unpacked(self):
        self.maxMechanicalSpeed = struct.unpack('<I',bytes(self.buffer[0:4]))[0]    #uint32_t
        self.maxReadableCurrent = struct.unpack('<f',bytes(self.buffer[4:8]))[0]    # float_t 
        self.nominalCurrent=struct.unpack('<f',bytes(self.buffer[8:12]))[0] #float_t
        self.nominalVoltage=struct.unpack('<H',bytes(self.buffer[12:14]))[0] #uint16_t
        self.driveType=self.buffer[14]  #uint8_t
        self.padding=self.buffer[15]    #uint8_t
        
    def Printdata(self):
        print(self.generate_application_config_string())
        
    def generate_application_config_string(self):
        output = []
        output.append("---Aplication_Config---")
        output.append(f"maxMechanicalSpeed : {self.maxMechanicalSpeed}")
        output.append(f"maxReadableCurrent : {self.maxReadableCurrent}")
        output.append(f"nominalCurrent : {self.nominalCurrent}")
        output.append(f"nominalVoltage : {self.nominalVoltage}")
        output.append(f"driveType : {self.DriveType()}")
        output.append(f"padding : {self.padding}")
        output.append("")  # 추가적인 빈 줄

        return "\n".join(output)
            
        

class MotorConfig_reg_t:
    def __init__(self):
        self.update = False
        self.buffer = list()
        self.size = 9 * 4 + 24
        self.polePairs = 0      # float_t 
        self.ratedFlux = 0      # float_t
        self.rs = 0             # float_t
        self.rsSkinFactor = 0   # float_t
        self.ls = 0             # float_t
        self.ld = 0             # float_t
        self.maxCurrent = 1     # float_t
        self.mass_copper_kg = 0 # float_t
        self.cooling_tau_s = 0  # float_t
        self.name = list()      # char_t
        
    def Unpacked(self):
        self.polePairs = struct.unpack('<f',bytes(self.buffer[0:4]))[0]      # float_t 
        self.ratedFlux = struct.unpack('<f',bytes(self.buffer[4:8]))[0]      # float_t
        self.rs = struct.unpack('<f',bytes(self.buffer[8:12]))[0]             # float_t
        self.rsSkinFactor = struct.unpack('<f',bytes(self.buffer[12:16]))[0]   # float_t
        self.ls = struct.unpack('<f',bytes(self.buffer[16:20]))[0]             # float_t
        self.ld = struct.unpack('<f',bytes(self.buffer[20:24]))[0]             # float_t
        self.maxCurrent = struct.unpack('<f',bytes(self.buffer[24:28]))[0]     # float_t
        self.mass_copper_kg = struct.unpack('<f',bytes(self.buffer[32:36]))[0] # float_t
        self.cooling_tau_s = struct.unpack('<f',bytes(self.buffer[36:40]))[0]  # float_t
        self.name = bytes(self.buffer[40:]).decode('ascii')     # char_t
        
    def Printdata(self):
        print(self.generate_motor_config_string)
        
    def generate_motor_config_string(self):
        output = []
        output.append("---MotorConfig_reg_t---")
        output.append(f"polePairs : {self.polePairs:.5f}")
        output.append(f"ratedFlux : {self.ratedFlux:.5f}")
        output.append(f"rs : {self.rs:.5f}")
        output.append(f"ls : {self.ls:.5f}")
        output.append(f"ld : {self.ld:.5f}")
        output.append(f"maxCurrent : {self.maxCurrent:.5f}")
        output.append(f"mass_copper_kg : {self.mass_copper_kg:.5f}")
        output.append(f"cooling_tau_s : {self.cooling_tau_s:.5f}")
        output.append(f"name : {self.name}")
        output.append("")

        return "\n".join(output)
        
class FOCFwConfig_reg_t:
    def __init__(self):
        self.update = False
        self.buffer = list()
        self.size = 1 * 4 + 4 + 2 * 3
        self.primarySensor = 0          # uint8_t
        self.auxiliarySensor = 0        # uint8_t
        self.topology = 0               # uint8_t
        self.FOCRate = 0                # uint8_t
        self.PWMFrequency = 0           # uint32_t
        self.MediumFrequency = 0        # uint16_t
        self.configurationFlag1 = 0     # uint16_t
        self.configurationFlag2 = 0     # uint16_t
        
    def Unpacked(self):
        self.primarySensor = self.buffer[0]                                         # uint8_t 
        self.auxiliarySensor = self.buffer[1]                                       # uint8_t
        self.topology = self.buffer[2]                                              # uint8_t
        self.FOCRate = self.buffer[3]                                               # uint8_t
        self.PWMFrequency = struct.unpack('<I',bytes(self.buffer[4:8]))[0]       # uint32_t
        self.MediumFrequency = struct.unpack('<H',bytes(self.buffer[8:10]))[0]    # uint16_t
        self.configurationFlag1 = struct.unpack('<H',bytes(self.buffer[10:12]))[0] # uint16_t
        self.configurationFlag2 = struct.unpack('<H',bytes(self.buffer[12:14]))[0] # uint16_t

    def Sensor(self, sensor):
        if sensor == Speed_Sensor.ENO_SENSOR.value:
            return "No Sensor"
        elif sensor == Speed_Sensor.EPLL.value:
            return "PILL"
        elif sensor == Speed_Sensor.ECORDIC.value:
            return "CORDIC"
        elif sensor == Speed_Sensor.EENCODER.value:
            return "Encoder"
        elif sensor == Speed_Sensor.EHALL.value:
            return "Hall"
        elif sensor == Speed_Sensor.EHSO.value:
            return "HSO"
        elif sensor == Speed_Sensor.EZEST.value:
            return "ZEST"
        else:
            return "No Sensor"
        
    def Topology(self):
        if self.topology == 0:
            return "3 Shunt"
    
    
    def flux_weakening_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, FLUX_WEAKENING_FLAG, output_type)

    def feed_forward_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, FEED_FORWARD_FLAG, output_type)

    def mtp_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, MTPA_FLAG, output_type)

    def pfc_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, PFC_FLAG, output_type)

    def icl_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, ICL_FLAG, output_type)

    def resistive_break_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, RESISTIVE_BREAK_FLAG, output_type)

    def ocp_disabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, OCP_DISABLE_FLAG, output_type)

    def stgap_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, STGAP_FLAG, output_type)

    def position_ctrl_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, POSITION_CTRL_FLAG, output_type)

    def vbus_sensing_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, VBUS_SENSING_FLAG, output_type)

    def temp_sensing_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, TEMP_SENSING_FLAG, output_type)

    def voltage_sensing_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, VOLTAGE_SENSING_FLAG, output_type)

    def flash_config_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, FLASH_CONFIG_FLAG, output_type)

    def dac_ch1_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, DAC_CH1_FLAG, output_type)

    def dac_ch2_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, DAC_CH2_FLAG, output_type)

    def otf_startup_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag1, OTF_STARTUP_FLAG, output_type)

    def overmodulation_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag2, OVERMODULATION_FLAG, output_type)

    def discontinuous_pwm_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag2, DISCONTINUOUS_PWM_FLAG, output_type)

    def profiler_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag2, PROFILER_FLAG, output_type)

    def dbg_mcu_load_measure_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag2, DBG_MCU_LOAD_MEASURE_FLAG, output_type)

    def dbg_open_loop_enabled(self, output_type=None):
        return self._check_flag(self.configurationFlag2, DBG_OPEN_LOOP_FLAG, output_type)

    def _check_flag(self, flag_group, flag, output_type):
        is_enabled = (flag_group & flag) != 0
        if output_type is None:
            return is_enabled
        elif isinstance(output_type, bool):
            return is_enabled
        elif isinstance(output_type, str):
            return "Enable" if is_enabled else "Disable"
        else:
            raise ValueError("output_type must be either bool or str.")
    
    
    def Printdata(self):
        print(self.generate_focfw_config_string())
        
    def generate_focfw_config_string(self):
        output = []
        output.append("---FOCFwConfig_reg_t---")
        output.append(f"primarySensor : {self.Sensor(self.primarySensor)}")
        output.append(f"auxiliarySensor : {self.Sensor(self.auxiliarySensor)}")
        output.append(f"topology : {self.topology}")
        output.append(f"FOCRate : {self.FOCRate}")
        output.append(f"PWMFrequency : {self.PWMFrequency}")
        output.append(f"MediumFrequency : {self.MediumFrequency}")
        output.append(f"configurationFlag1 : {self.configurationFlag1}")
        output.append(f"configurationFlag2 : {self.configurationFlag2}")

        # configurationFlag1 상태 추가
        output.append(f"FLUX_WEAKENING_FLAG : {self.flux_weakening_enabled('status')}")
        output.append(f"FEED_FORWARD_FLAG : {self.feed_forward_enabled('status')}")
        output.append(f"MTPA_FLAG : {self.mtp_enabled('status')}")
        output.append(f"PFC_FLAG : {self.pfc_enabled('status')}")
        output.append(f"ICL_FLAG : {self.icl_enabled('status')}")
        output.append(f"RESISTIVE_BREAK_FLAG : {self.resistive_break_enabled('status')}")
        output.append(f"OCP_DISABLE_FLAG : {self.ocp_disabled('status')}")
        output.append(f"STGAP_FLAG : {self.stgap_enabled('status')}")
        output.append(f"POSITION_CTRL_FLAG : {self.position_ctrl_enabled('status')}")
        output.append(f"VBUS_SENSING_FLAG : {self.vbus_sensing_enabled('status')}")
        output.append(f"TEMP_SENSING_FLAG : {self.temp_sensing_enabled('status')}")
        output.append(f"VOLTAGE_SENSING_FLAG : {self.voltage_sensing_enabled('status')}")
        output.append(f"FLASH_CONFIG_FLAG : {self.flash_config_enabled('status')}")
        output.append(f"DAC_CH1_FLAG : {self.dac_ch1_enabled('status')}")
        output.append(f"DAC_CH2_FLAG : {self.dac_ch2_enabled('status')}")
        output.append(f"OTF_STARTUP_FLAG : {self.otf_startup_enabled('status')}")

        # configurationFlag2 상태 추가
        output.append(f"OVERMODULATION_FLAG : {self.overmodulation_enabled('status')}")
        output.append(f"DISCONTINUOUS_PWM_FLAG : {self.discontinuous_pwm_enabled('status')}")
        output.append(f"PROFILER_FLAG : {self.profiler_enabled('status')}")
        output.append(f"DBG_MCU_LOAD_MEASURE_FLAG : {self.dbg_mcu_load_measure_enabled('status')}")
        output.append(f"DBG_OPEN_LOOP_FLAG : {self.dbg_open_loop_enabled('status')}")
        output.append("")  # 추가적인 빈 줄

        return "\n".join(output)  # 리스트를 문자열로 결합하여 반환


        

class Motor_Param:
    def __init__(self):
        self.application_config = Aplication_Config()
        self.motor_config = MotorConfig_reg_t()
        self.focfw_config = FOCFwConfig_reg_t()
        self.power_stage = [False, ""]
        self.motor_name = [False, ""]

        # Defien Current Sensing Hardware Parameter
        self.RSHUNT = 0.003
        self.AMPLIFICATION_GAIN = 9.14
        self.ADC_REFERENCE_VOLTAGE = 3.3
        
        self.U_RPM = 60
        self.U_01HZ = 10
        self.SPEED_UNIT = 0
        self.SPEED_UNIT = self.U_01HZ

        
    def Max_Motor_Speed(self):
        return self.application_config.maxMechanicalSpeed
    
    def Max_Readable_Current(self):
        return self.application_config.maxReadableCurrent
    
    def Nominal_Current(self):
        return self.application_config.nominalCurrent
    
    def Nominal_Voltage(self):
        return self.application_config.nominalVoltage
    
    def Control_Type(self):
        return self.application_config.DriveType()
    
    def Primary_Speed_sensor(self):
        return self.focfw_config.Sensor(self.focfw_config.primarySensor)
    
    def Aux_Speed_Sensor(self):
        return self.focfw_config.Sensor(self.focfw_config.auxiliarySensor)
    
    def Current_Sensing_Topology(self):
        return self.focfw_config.Topology()
    
    def FOC_rate(self):
        return self.focfw_config.FOCRate
    
    def PWM_Frequency(self):
        return self.focfw_config.PWMFrequency
    
    def Medium_Frequency(self):
        return self.focfw_config.MediumFrequency
    
    def Feed_Forward(self, type):
        return self.focfw_config.feed_forward_enabled(type)
    
    def Position_Control(self, type):
        return self.focfw_config.position_ctrl_enabled(type)
    
    def VBus_Sensing(self, type):
        return self.focfw_config.vbus_sensing_enabled(type)
    
    def Temperature_Sensing(self,type):
        return self.focfw_config.temp_sensing_enabled(type)
    
    def Current_Conv_Factor(self):
        return ((65536.0 * self.RSHUNT * self.AMPLIFICATION_GAIN)/self.ADC_REFERENCE_VOLTAGE)
    
    def IQMAX(self):
        return self.motor_config.maxCurrent * self.Current_Conv_Factor()
    
    def MAX_APPLICATION_SPEED_UNIT(self):
        return  int(int(self.Max_Motor_Speed() * self.SPEED_UNIT) / self.U_RPM)
    
    def RPM2SPEEDUNIT(self, rpm):
        return int(rpm * self.SPEED_UNIT / self.U_RPM)
    
    def SPEEDUNIT2RPM(self, speedunit):
        return int(speedunit * self.U_RPM / self.SPEED_UNIT)

if __name__ == "__main__" :
    mtor = Motor_Param()