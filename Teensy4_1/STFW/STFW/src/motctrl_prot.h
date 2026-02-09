#ifndef _MOTCTRL_PROT_H__
#define _MOTCTRL_PROT_H__
#define MOTCTRL_FRAME_SIZE 8
#include <stdint.h>
#include <stdbool.h>



typedef enum {
  MOTCTRL_CMD_RESET_CONFIGURATION = 0x81,
  MOTCTRL_CMD_REFRESH_CONFIGURATION = 0x82,
  MOTCTRL_CMD_MODIFY_CONFIGURATION = 0x83,
  MOTCTRL_CMD_RETRIEVE_CONFIGURATION = 0x84,
  MOTCTRL_CMD_START_MOTOR = 0x91,
  MOTCTRL_CMD_STOP_MOTOR = 0x92,
  MOTCTRL_CMD_TORQUE_CONTROL = 0x93,
  MOTCTRL_CMD_SPEED_CONTROL = 0x94,
  MOTCTRL_CMD_POSITION_CONTROL = 0x95,
  MOTCTRL_CMD_PTS_CONTROL = 0x96,
  MOTCTRL_CMD_STOP_CONTROL = 0x97,
  MOTCTRL_CMD_MODIFY_PARAMETER = 0xA1,
  MOTCTRL_CMD_RETRIEVE_PARAMETER = 0xA2,
  MOTCTRL_CMD_GET_VERSION = 0xB1,
  MOTCTRL_CMD_GET_FAULT = 0xB2,
  MOTCTRL_CMD_ACK_FAULT = 0xB3,
  MOTCTRL_CMD_RETRIEVE_INDICATOR = 0xB4,
} MOTCTRL_CMD;

/**
 * command execution result
*/
typedef enum {
  MOTCTRL_RES_SUCCESS = 0,
  MOTCTRL_RES_FAIL = 1,
  MOTCTRL_RES_FAIL_UNKNOWN_CMD = 2,
  MOTCTRL_RES_FAIL_UNKNOWN_ID = 3,
  MOTCTRL_RES_FAIL_RO_REG = 4,
  MOTCTRL_RES_FAIL_UNKNOWN_REG = 5,
  MOTCTRL_RES_FAIL_STR_FORMAT = 6,
  MOTCTRL_RES_FAIL_DATA_FORMAT = 7,
  MOTCTRL_RES_FAIL_WO_REG = 0xB,
  MOTCTRL_RES_FAIL_NOT_CONNECTED = 0x80,
} MOTCTRL_RES;

/**
 * configuration item value type
*/
typedef enum {
  MOTCTRL_CONFTYPE_INT = 0,
  MOTCTRL_CONFTYPE_FLOAT = 1,
} MOTCTRL_CONFTYPE;

/**
 * configuration id for int type
*/
typedef enum {
  MOTCTRL_CONFID_INT_POLE_PAIRS = 0x00,               // Pole Pairs
  MOTCTRL_CONFID_INT_RATED_CURRENT = 0x01,            // Rated Current (A)
  MOTCTRL_CONFID_INT_MAX_SPEED = 0x02,                // Max Speed (RPM)
  MOTCTRL_CONFID_INT_RATED_VOLTAGE = 0x06,            // Rated Voltage (V)
  MOTCTRL_CONFID_INT_PWM_FREQ = 0x07,                 // PWM Frequency (Hz)
  MOTCTRL_CONFID_INT_TORQUE_KP_DEFAULT = 0x08,        // Default KP of Current Loop
  MOTCTRL_CONFID_INT_TORQUE_KI_DEFAULT = 0x09,        // Default KI of Current Loop
  MOTCTRL_CONFID_INT_SPEED_KP_DEFAULT = 0x0C,         // Default KP of Speed Loop
  MOTCTRL_CONFID_INT_SPEED_KI_DEFAULT = 0x0D,         // Default KI of Speed Loop
  MOTCTRL_CONFID_INT_POSITION_KP_DEFAULT = 0x0E,      // Default KP of Position Loop
  MOTCTRL_CONFID_INT_POSITION_KI_DEFAULT = 0x0F,      // Default KI of Position Loop
  MOTCTRL_CONFID_INT_POSITION_KD_DEFAULT = 0x10,      // Default KD of Position Loop
  MOTCTRL_CONFID_INT_GEAR_RATIO = 0x11,               // Gear Ratio
  MOTCTRL_CONFID_INT_CAN_ID = 0x12,                   // CAN ID
  MOTCTRL_CONFID_INT_CAN_MASTER_ID = 0x13,            // Master(Host) CAN ID
  MOTCTRL_CONFID_INT_ZERO_POSITION = 0x14,            // Zero Position (for Output Shaft)
  MOTCTRL_CONFID_INT_POWEROFF_POSITION = 0x15,        // (OBSOLETE)Power-Off Position (for Output Shaft)
  MOTCTRL_CONFID_INT_OV_THRESHOLD = 0x16,             // Over Voltage Threshold (V)
  MOTCTRL_CONFID_INT_UV_THRESHOLD = 0x17,             // Under Voltage Threshold (V)
  MOTCTRL_CONFID_INT_CAN_BAUDRATE = 0x18,             // CAN Baud Rate
  MOTCTRL_CONFID_INT_FW_KP_DEFAULT = 0x19,            // Default KP of Flux Weakening
  MOTCTRL_CONFID_INT_FW_KI_DEFAULT = 0x1A,            // Default KI of Flux Weakening
  MOTCTRL_CONFID_INT_OV_TEMP_THRESHOLD = 0x20,        // Over Temperature Threshold (in Centigrade)
  MOTCTRL_CONFID_INT_CAN_PROT = 0x1C,                 // Protocol over CAN, @ref MOTCTRL_CONF_CAN_PROT
} MOTCTRL_CONFID_INT;

/**
 * configuration id for float type
*/
typedef enum {
  MOTCTRL_CONFID_FLOAT_RS = 0x00,                     // Rs (Ω)
  MOTCTRL_CONFID_FLOAT_LS = 0x01,                     // Ls (H)
  MOTCTRL_CONFID_FLOAT_BEMF_CONST = 0x02,             // Back EMF Constant
  MOTCTRL_CONFID_FLOAT_TORQUE_CONST = 0x03,           // Torque Constant (N.m/A)
  MOTCTRL_CONFID_FLOAT_SAMPLING_RESISTOR = 0x04,      // Sampling Resistor (Ω)
  MOTCTRL_CONFID_FLOAT_AMP_GAIN = 0x05,               // Amplification Gain
} MOTCTRL_CONFID_FLOAT;

#define MOTCTRL_CONFID uint8_t

/**
 * protocol type over CAN
*/
typedef enum {
  MOTCTRL_CONF_CAN_PROT_CYBERBEAST = 0,               // CyberBeast
  MOTCTRL_CONF_CAN_PROT_MIT = 1,                      // MIT
} MOTCTRL_CONF_CAN_PROT;

/**
 * runtime parameter id
*/
typedef enum {
  MOTCTRL_PARAID_TORQUE_KP = 0x00,
  MOTCTRL_PARAID_TORQUE_KI = 0x01,
  MOTCTRL_PARAID_SPEED_KP = 0x02,
  MOTCTRL_PARAID_SPEED_KI = 0x03,
  MOTCTRL_PARAID_POSITION_KP = 0x04,
  MOTCTRL_PARAID_POSITION_KI = 0x05,
  MOTCTRL_PARAID_POSITION_KD = 0x06,
  MOTCTRL_PARAID_FW_KP = 0x07,
  MOTCTRL_PARAID_FW_KI = 0x08,
} MOTCTRL_PARAID;

/**
 * fault numbers
*/
typedef enum {
  MOTCTRL_FAULTNO_NONE = 0x00,
  MOTCTRL_FAULTNO_FREQ_TOO_HIGH = 0x01,
  MOTCTRL_FAULTNO_OV = 0x02,
  MOTCTRL_FAULTNO_UV = 0x04,
  MOTCTRL_FAULTNO_OT = 0x08,
  MOTCTRL_FAULTNO_START_FAIL = 0x10,
  MOTCTRL_FAULTNO_OC = 0x40,
  MOTCTRL_FAULTNO_SOFTWARE_EXCEPTION = 0x80,
} MOTCTRL_FAULTNO;

/**
 * runtime indicator id
*/
typedef enum {
  MOTCTRL_INDIID_BUS_VOLTAGE = 0x00,
  MOTCTRL_INDIID_TEMP_BOARD = 0x01,
  MOTCTRL_INDIID_TEMP_MOTOR = 0x02,
  MOTCTRL_INDIID_POWER = 0x03,
  MOTCTRL_INDIID_IA = 0x04,
  MOTCTRL_INDIID_IB = 0x05,
  MOTCTRL_INDIID_IC = 0x06,
  MOTCTRL_INDIID_IALPHA = 0x07,
  MOTCTRL_INDIID_IBETA = 0x08,
  MOTCTRL_INDIID_IQ = 0x09,
  MOTCTRL_INDIID_ID = 0x0A,
  MOTCTRL_INDIID_TARGET_IQ = 0x0B,
  MOTCTRL_INDIID_TARGET_ID = 0x0C,
  MOTCTRL_INDIID_VQ = 0x0D,
  MOTCTRL_INDIID_VD = 0x0E,
  MOTCTRL_INDIID_VALPHA = 0x0F,
  MOTCTRL_INDIID_VBETA = 0x10,
  MOTCTRL_INDIID_EL_ANGLE_ROTOR = 0x11,
  MOTCTRL_INDIID_MEC_ANGLE_ROTOR = 0x12,
  MOTCTRL_INDIID_MEC_ANGLE_SHAFT = 0x13,
  MOTCTRL_INDIID_SPEED_SHAFT = 0x14,
  MOTCTRL_INDIID_OUTPUT_POWER = 0x15,
} MOTCTRL_INDIID;

/**
 * @brief reset all configurations to default values
 * @param reqBuf command message buffer
*/
void MCReqResetConfiguration(uint8_t * reqBuf);
/**
 * @brief unpack the reset configuration response message
 * @param resBuf response meesage buffer
 * @return result of the command execution
*/
MOTCTRL_RES MCResResetConfiguration(uint8_t * resBuf);

/**
 * @brief refresh all configurations
 * @param reqBuf command message buffer
*/
void MCReqRefreshConfiguration(uint8_t * reqBuf);
/**
 * @brief unpack the refresh configution response message
 * @param resBuf response meesage buffer
 * @return result of the command execution
*/
MOTCTRL_RES MCResRefreshConfiguration(uint8_t * resBuf);

/**
 * @brief modify single configuration
 * @param reqBuf command message buffer
 * @param confType the configuration item data type
 * @param confID the configuration item id
 * @param confData the configuration data to be updated
*/
void MCReqModifyConfiguration(uint8_t * reqBuf, MOTCTRL_CONFTYPE confType, MOTCTRL_CONFID confID, float confData);
/**
 * @brief unpack the modify configuration response message
 * @param resBuf response meesage buffer
 * @param confType the configuration item data type
 * @param confID the configuration item id
 * @return result of the command execution
*/
MOTCTRL_RES MCResModifyConfiguration(uint8_t * resBuf, MOTCTRL_CONFTYPE * confType, MOTCTRL_CONFID * confID);

/**
 * @brief retrieve single configuration
 * @param reqBuf command message buffer
 * @param confType the configuration item data type
 * @param confID the configuration item id
*/
void MCReqRetrieveConfiguration(uint8_t * reqBuf, MOTCTRL_CONFTYPE confType, MOTCTRL_CONFID confID);
/**
 * @brief unpack the retrieve configuration response message
 * @param resBuf response meesage buffer
 * @param confType the configuration item data type
 * @param confID the configuration item id
 * @param confData the configuration data retrieved
 * @return result of the command execution
*/
MOTCTRL_RES MCResRetrieveConfiguration(uint8_t * resBuf, MOTCTRL_CONFTYPE * confType, MOTCTRL_CONFID * confID, float * confData);

/**
 * @brief start the motor
 * @param reqBuf command message buffer
*/
void MCReqStartMotor(uint8_t * reqBuf);
/**
 * @brief unpack the start motor response message
 * @param resBuf response meesage buffer
 * @return result of the command execution
*/
MOTCTRL_RES MCResStartMotor(uint8_t * resBuf);

/**
 * @brief stop the motor
 * @param reqBuf command message buffer
*/
void MCReqStopMotor(uint8_t * reqBuf);
/**
 * @brief unpack the stop motor response message
 * @param resBuf response meesage buffer
 * @return result of the command execution
*/
MOTCTRL_RES MCResStopMotor(uint8_t * resBuf);

/**
 * @brief encapsulate torque control command message
 * @param reqBuf command message buffer
 * @param torque in N.m
 * @param duration in ms
*/
void MCReqTorqueControl(uint8_t * reqBuf, float torque);
/**
 * @brief unpack the torque control response message
 * @param resBuf response meesage buffer
 * @param temp current temperature
 * @param position current position in RAD for output shaft
 * @param speed current speed in RAD/s for output shaft
 * @param torque current torque in Amper, multiply by torque constant and gear ratio to get the torque in N.m for output shaft 
 * @return result of the command execution
*/
MOTCTRL_RES MCResTorqueControl(uint8_t * resBuf, int8_t * temp, float * position, float * speed, float * torque);

/**
 * @brief encapsulate speed control command message
 * @param reqBuf command message buffer
 * @param speed in RAD/s
 * @param duration in ms
*/
void MCReqSpeedControl(uint8_t * reqBuf, float speed);
/**
 * @brief unpack the speed control response message
 * @param resBuf response meesage buffer
 * @param temp current temperature
 * @param position current position in RAD for output shaft
 * @param speed current speed in RAD/s for output shaft
 * @param torque current torque in Amper, multiply by torque constant and gear ratio to get the torque in N.m for output shaft 
 * @return result of the command execution
*/
MOTCTRL_RES MCResSpeedControl(uint8_t * resBuf, int8_t * temp, float * position, float * speed, float * torque);
/**
 * @brief encapsulate position control command message
 * @param reqBuf command message buffer
 * @param position in RAD
 * @param duration in ms
*/
void MCReqPositionControl(uint8_t * reqBuf, float position);
/**
 * @brief unpack the position control response message
 * @param resBuf response meesage buffer
 * @param temp current temperature
 * @param position current position in RAD for output shaft
 * @param speed current speed in RAD/s for output shaft
 * @param torque current torque in Amper, multiply by torque constant and gear ratio to get the torque in N.m for output shaft 
 * @return result of the command execution
*/
MOTCTRL_RES MCResPositionControl(uint8_t * resBuf, int8_t * temp, float * position, float * speed, float * torque);



/**
 * @brief Unpack a motor control response message (position/speed/torque control)
 * @param resBuf  Response message buffer (raw bytes from controller)
 * @param temp    Current temperature
 * @param position Current position in RAD for output shaft
 * @param speed    Current speed in RAD/s for output shaft
 * @param torque   Current torque in Amper; multiply by torque constant and gear ratio to get N·m at output shaft
 * @return Result of the command execution (MOTCTRL_RES_OK or MOTCTRL_RES_FAIL)
 */
MOTCTRL_RES MCResControl(uint8_t *resBuf, int8_t *temp, float *position, float *speed, float *torque);


/**
 * @brief stop current control process
 * @param reqBuf command message buffer
*/
void MCReqStopControl(uint8_t * reqBuf);
/**
 * @brief unpack the stop control response message
 * @param resBuf response meesage buffer
 * @return result of the command execution
*/
MOTCTRL_RES MCResStopControl(uint8_t * resBuf);

/**
 * @brief modify single parameter
 * @param reqBuf command message buffer
 * @param paraID parameter id
 * @param paraData parameter data to be updated
*/
void MCReqModifyParameter(uint8_t * reqBuf, MOTCTRL_PARAID paraID, float paraData);
/**
 * @brief unpack the modify parameter response message
 * @param resBuf response meesage buffer
 * @param paraID parameter id
 * @return result of the command execution
*/
MOTCTRL_RES MCResModifyParameter(uint8_t * resBuf, MOTCTRL_PARAID * paraID);

/**
 * @brief retrieve single parameter
 * @param reqBuf command message buffer
 * @param paraID parameter id
*/
void MCReqRetrieveParameter(uint8_t * reqBuf, MOTCTRL_PARAID paraID);
/**
 * @brief unpack the retrieve parameter response message
 * @param resBuf response meesage buffer
 * @param paraID parameter id
 * @param paraData parameter data retrieved
 * @return result of the command execution
*/
MOTCTRL_RES MCResRetrieveParameter(uint8_t * resBuf, MOTCTRL_PARAID * paraID, float * paraData);

/**
 * @brief get the current version
 * @param reqBuf command message buffer
*/
void MCReqGetVersion(uint8_t * reqBuf);
/**
 * @brief unpack the get version response message
 * @param resBuf response meesage buffer
 * @param version the current version
 * @return result of the command execution
*/
MOTCTRL_RES MCResGetVersion(uint8_t * resBuf, uint32_t * version);

/**
 * @brief get fault status
 * @param reqBuf command message buffer
*/
void MCReqGetFault(uint8_t * reqBuf);
/**
 * @brief unpack the get fault response message
 * @param resBuf response meesage buffer
 * @param faultNo the current fault No.
 * @return result of the command execution
*/
MOTCTRL_RES MCResGetFault(uint8_t * resBuf, MOTCTRL_FAULTNO * faultNo);

/**
 * @brief acknowlege (eliminate) current fault status if any
 * @param reqBuf command message buffer
*/
void MCReqAckFault(uint8_t * reqBuf);
/**
 * @brief unpack the acknowledge fault response message
 * @param resBuf response meesage buffer
 * @return result of the command execution
*/
MOTCTRL_RES MCResAckFault(uint8_t * resBuf);

/**
 * @brief retrieve single runtime indicator
 * @param reqBuf command message buffer
 * @param indiID indicator id
*/
void MCReqRetrieveIndicator(uint8_t * reqBuf, MOTCTRL_INDIID indiID);
/**
 * @brief unpack the retrieve indicator response message
 * @param resBuf response meesage buffer
 * @param indiID indicator id
 * @param indiData indicator data retrieved
 * @return result of the command execution
*/
MOTCTRL_RES MCResRetrieveIndicator(uint8_t * resBuf, MOTCTRL_INDIID * indiID, float * indiData);

void SendCmd2Motor(uint8_t * buf);

void ReceiveResFromMotor(uint8_t * buf);

#endif // MOTCTRL_PROT_H

