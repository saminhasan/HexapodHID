#include "motctrl_prot.h"
#include <stdbool.h>
// WARN : Little Endian

void MCReqResetConfiguration(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_RESET_CONFIGURATION;
  return;
}

MOTCTRL_RES MCResResetConfiguration(uint8_t * resBuf)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_RESET_CONFIGURATION) {
    return MOTCTRL_RES_FAIL;
  }
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqRefreshConfiguration(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_REFRESH_CONFIGURATION;
  return;
}

MOTCTRL_RES MCResRefreshConfiguration(uint8_t * resBuf)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_REFRESH_CONFIGURATION) {
    return MOTCTRL_RES_FAIL;
  }
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqModifyConfiguration(uint8_t * reqBuf, MOTCTRL_CONFTYPE confType, MOTCTRL_CONFID confID, float confData)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_MODIFY_CONFIGURATION;
  reqBuf[1] = (uint8_t)confType;
  reqBuf[2] = (uint8_t)confID;

  uint8_t * confDataPtr = 0;
  int32_t confDataInt = (int32_t)confData;
  switch (confType) {
    default:
    case MOTCTRL_CONFTYPE_INT: {
      confDataPtr = (uint8_t *)(&confDataInt);
      break;
    }
    case MOTCTRL_CONFTYPE_FLOAT: {
      confDataPtr = (uint8_t *)(&confData);
      break;
    }
  }
    reqBuf[4] = confDataPtr[0];
    reqBuf[5] = confDataPtr[1];
    reqBuf[6] = confDataPtr[2];
    reqBuf[7] = confDataPtr[3];
}

MOTCTRL_RES MCResModifyConfiguration(uint8_t * resBuf, MOTCTRL_CONFTYPE * confType, MOTCTRL_CONFID * confID)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_MODIFY_CONFIGURATION) {
    return MOTCTRL_RES_FAIL;
  }
  * confType = (MOTCTRL_CONFTYPE)resBuf[1];
  * confID = (MOTCTRL_CONFID)resBuf[2];
  return (MOTCTRL_RES)resBuf[3];
}

void MCReqRetrieveConfiguration(uint8_t * reqBuf, MOTCTRL_CONFTYPE confType, MOTCTRL_CONFID confID)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_RETRIEVE_CONFIGURATION;
  reqBuf[1] = (uint8_t)confType;
  reqBuf[2] = (uint8_t)confID;
}

MOTCTRL_RES MCResRetrieveConfiguration(uint8_t * resBuf, MOTCTRL_CONFTYPE * confType, MOTCTRL_CONFID * confID, float * confData)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_RETRIEVE_CONFIGURATION) {
    return MOTCTRL_RES_FAIL;
  }
  *confType = (MOTCTRL_CONFTYPE)resBuf[1];
  *confID = (MOTCTRL_CONFID)resBuf[2];

  uint8_t confDataPtr[4];

    confDataPtr[0] = resBuf[4];
    confDataPtr[1] = resBuf[5];
    confDataPtr[2] = resBuf[6];
    confDataPtr[3] = resBuf[7];
  switch (*confType) {
    default:
    case MOTCTRL_CONFTYPE_INT: {
      int32_t * confDataInt = (int32_t *)confDataPtr;
      *confData = (float)(*confDataInt);
      break;
    }
    case MOTCTRL_CONFTYPE_FLOAT: {
      float * confDataFloat = (float *)confDataPtr;
      *confData = *confDataFloat;
      break;
    }
  }
  return (MOTCTRL_RES)resBuf[3];
}

void MCReqStartMotor(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_START_MOTOR;
}

MOTCTRL_RES MCResStartMotor(uint8_t * resBuf)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_START_MOTOR) {
    return MOTCTRL_RES_FAIL;
  }
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqStopMotor(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_STOP_MOTOR;
}

MOTCTRL_RES MCResStopMotor(uint8_t * resBuf)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_STOP_MOTOR) {
    return MOTCTRL_RES_FAIL;
  }
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqTorqueControl(uint8_t * reqBuf, float torque)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_TORQUE_CONTROL;
  uint8_t * torquePtr = (uint8_t *)(&torque);
  // uint8_t * durationPtr = (uint8_t *)(&duration);

    reqBuf[1] = torquePtr[0];
    reqBuf[2] = torquePtr[1];
    reqBuf[3] = torquePtr[2];
    reqBuf[4] = torquePtr[3];

    reqBuf[5] = 0;//durationPtr[0];
    reqBuf[6] = 0;//durationPtr[1];
    reqBuf[7] = 0;//durationPtr[2];
  
}

MOTCTRL_RES MCResTorqueControl(uint8_t * resBuf, int8_t * temp, float * position, float * speed, float * torque)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_TORQUE_CONTROL) {
    return MOTCTRL_RES_FAIL;
  }
  * temp = resBuf[2];
  uint16_t pos_int;
  uint8_t * tmp = (uint8_t *)&pos_int;
  tmp[0] = resBuf[3];
  tmp[1] = resBuf[4];
  * position = (float)pos_int * 25 / 65535 - 12.5; // in RAD, between -12.5 ~ 12.5
  int16_t speed_int = (int16_t)(((uint16_t)resBuf[5] << 4) | (resBuf[6] >> 4));
  * speed = (float)speed_int * 130 / 4095 - 65; // in RAD/s, between -65 ~ 65
  int16_t torque_int = (int16_t)(((uint16_t)(resBuf[6] & 0x0F) << 8) | resBuf[7]);
  // * torque = (float)torque_int * 450 / 4095 - 225; // in Amper, between -225 ~ 225
  // torque_float = torque_int * (450 * torque_constant * gear_ratio) / 4095– 225 * torque_constant * gear_ratio
  //torque_constant , gear_ratio = 0.116670, 9
  * torque = (float)torque_int * (450 * 0.116670 * 9.0) / 4095 - 225 * 0.116670 * 9.0;

  return (MOTCTRL_RES)resBuf[1];
}

void MCReqSpeedControl(uint8_t * reqBuf, float speed)
{
  if (reqBuf == 0) {
    return;
  }
  speed = (9.54929658551f)*speed; // convert RAD/s to RPM, driver expects RPM, but api is for RAD/s
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_SPEED_CONTROL;
	uint8_t * speedPtr = (uint8_t *)(&speed);
  // uint8_t * durationPtr = (uint8_t *)(&duration);

    reqBuf[1] = speedPtr[0];
    reqBuf[2] = speedPtr[1];
    reqBuf[3] = speedPtr[2];
    reqBuf[4] = speedPtr[3];

    reqBuf[5] = 0;//durationPtr[0];
    reqBuf[6] = 0;//durationPtr[1];
    reqBuf[7] = 0;//durationPtr[2];
  
}

MOTCTRL_RES MCResSpeedControl(uint8_t * resBuf, int8_t * temp, float * position, float * speed, float * torque)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_SPEED_CONTROL) {
    return MOTCTRL_RES_FAIL;
  }
  *temp = resBuf[2];
	uint16_t pos_int;
	uint8_t *tmp = (uint8_t *)&pos_int;
	tmp[0] = resBuf[3];
	tmp[1] = resBuf[4];
  *position = (float)pos_int * 25 / 65535 - 12.5; // in RAD, between -12.5 ~ 12.5
  int16_t speed_int = (int16_t)(((uint16_t)resBuf[5] << 4) | (resBuf[6] >> 4));
  *speed = (float)speed_int * 130 / 4095 - 65; // in RAD/s, between -65 ~ 65
  int16_t torque_int = (int16_t)(((uint16_t)(resBuf[6] & 0x0F) << 8) | resBuf[7]);
  // *torque = (float)torque_int * 450 / 4095 - 225; // in Amper, between -225 ~ 225
  // torque_float = torque_int * (450 * torque_constant * gear_ratio) / 4095– 225 * torque_constant * gear_ratio
  //torque_constant , gear_ratio = 0.116670, 9
  * torque = (float)torque_int * (450 * 0.116670 * 9.0) / 4095 - 225 * 0.116670 * 9.0;
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqPositionControl(uint8_t * reqBuf, float position)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_POSITION_CONTROL;
  uint8_t * posPtr = (uint8_t *)(&position);
  // uint8_t * durationPtr = (uint8_t *)(&duration);

    reqBuf[1] = posPtr[0];
    reqBuf[2] = posPtr[1];
    reqBuf[3] = posPtr[2];
    reqBuf[4] = posPtr[3];

    reqBuf[5] = 0;//durationPtr[0];
    reqBuf[6] = 0;//durationPtr[1];
    reqBuf[7] = 0;//durationPtr[2];
}

MOTCTRL_RES MCResPositionControl(uint8_t * resBuf, int8_t * temp, float * position, float * speed, float * torque)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_POSITION_CONTROL) {
    return MOTCTRL_RES_FAIL;
  }
  * temp = resBuf[2];
  uint16_t pos_int;
	uint8_t *tmp = (uint8_t *)&pos_int;
	tmp[0] = resBuf[3];
  tmp[1] = resBuf[4];
  * position = (float)pos_int * 25 / 65535 - 12.5; // in RAD, between -12.5 ~ 12.5
  int16_t speed_int = (int16_t)(((uint16_t)resBuf[5] << 4) | (resBuf[6] >> 4));
  * speed = (float)speed_int * 130 / 4095 - 65; // in RAD/s, between -65 ~ 65
  int16_t torque_int = (int16_t)(((uint16_t)(resBuf[6] & 0x0F) << 8) | resBuf[7]);
  // * torque = (float)torque_int * 450 / 4095 - 225; // in Amper, between -225 ~ 225
  // torque_float = torque_int * (450 * torque_constant * gear_ratio) / 4095– 225 * torque_constant * gear_ratio
  //torque_constant , gear_ratio = 0.116670, 9
  * torque = (float)torque_int * (450 * 0.116670 * 9.0) / 4095 - 225 * 0.116670 * 9.0;
  return (MOTCTRL_RES)resBuf[1];
}

MOTCTRL_RES MCResControl(uint8_t *resBuf, int8_t *temp, float *position, float *speed, float *torque)
{
  if (resBuf == 0) {
    return MOTCTRL_RES_FAIL;
  }
  uint8_t cmd = resBuf[0];
  if (cmd != MOTCTRL_CMD_TORQUE_CONTROL &&
      cmd != MOTCTRL_CMD_SPEED_CONTROL &&
      cmd != MOTCTRL_CMD_POSITION_CONTROL) {
    return MOTCTRL_RES_FAIL;
  }

  *temp = resBuf[2];

  uint16_t pos_int;
  uint8_t *tmp = (uint8_t *)&pos_int;
  tmp[0] = resBuf[3];
  tmp[1] = resBuf[4];
  *position = (float)pos_int * 25 / 65535 - 12.5; // RAD, -12.5..12.5

  int16_t speed_int = (int16_t)(((uint16_t)resBuf[5] << 4) | (resBuf[6] >> 4));
  *speed = (float)speed_int * 130 / 4095 - 65;     // RAD/s, -65..65

  int16_t torque_int = (int16_t)(((uint16_t)(resBuf[6] & 0x0F) << 8) | resBuf[7]);
  // torque_float = torque_int * (450 * torque_constant * gear_ratio) / 4095 – 225 * torque_constant * gear_ratio
  // torque_constant, gear_ratio = 0.116670, 9
  *torque = (float)torque_int * (450 * 0.116670f * 9.0f) / 4095 - 225 * 0.116670f * 9.0f;

  return (MOTCTRL_RES)resBuf[1];
}

void MCReqStopControl(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_STOP_CONTROL;
}

MOTCTRL_RES MCResStopControl(uint8_t * resBuf)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_STOP_CONTROL) {
    return MOTCTRL_RES_FAIL;
  }
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqModifyParameter(uint8_t * reqBuf, MOTCTRL_PARAID paraID, float paraData)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_MODIFY_PARAMETER;
  reqBuf[1] = (uint8_t)paraID;

  uint8_t * paraDataPtr = (uint8_t *)(&paraData);

    reqBuf[4] = paraDataPtr[0];
    reqBuf[5] = paraDataPtr[1];
    reqBuf[6] = paraDataPtr[2];
    reqBuf[7] = paraDataPtr[3];
}

MOTCTRL_RES MCResModifyParameter(uint8_t * resBuf, MOTCTRL_PARAID * paraID)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_MODIFY_PARAMETER) {
    return MOTCTRL_RES_FAIL;
  }
  *paraID = (MOTCTRL_PARAID)resBuf[1];
  return (MOTCTRL_RES)resBuf[3];
}

void MCReqRetrieveParameter(uint8_t * reqBuf, MOTCTRL_PARAID paraID)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_RETRIEVE_PARAMETER;
  reqBuf[1] = (uint8_t)paraID;
}

MOTCTRL_RES MCResRetrieveParameter(uint8_t * resBuf, MOTCTRL_PARAID * paraID, float * paraData)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_RETRIEVE_PARAMETER) {
    return MOTCTRL_RES_FAIL;
  }
  *paraID = (MOTCTRL_PARAID)resBuf[1];

  uint8_t paraDataPtr[4];

    paraDataPtr[0] = resBuf[4];
    paraDataPtr[1] = resBuf[5];
    paraDataPtr[2] = resBuf[6];
    paraDataPtr[3] = resBuf[7];
  
  int32_t *paraDataFloat = (int32_t *)paraDataPtr;
  *paraData = *paraDataFloat;
  return (MOTCTRL_RES)resBuf[2];
}

void MCReqGetVersion(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_GET_VERSION;
}

MOTCTRL_RES MCResGetVersion(uint8_t * resBuf, uint32_t * version)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_GET_VERSION) {
    return MOTCTRL_RES_FAIL;
  }

  uint8_t versionPtr[4];

    versionPtr[0] = resBuf[4];
    versionPtr[1] = resBuf[5];
    versionPtr[2] = resBuf[6];
    versionPtr[3] = resBuf[7];
  
  uint32_t * versionInt = (uint32_t *)versionPtr;
  *version = *versionInt;
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqGetFault(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_GET_FAULT;
}

MOTCTRL_RES MCResGetFault(uint8_t * resBuf, MOTCTRL_FAULTNO * faultNo)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_GET_FAULT) {
    return MOTCTRL_RES_FAIL;
  }

  *faultNo = (MOTCTRL_FAULTNO)resBuf[2];
  return (MOTCTRL_RES)resBuf[1];
}

void MCReqAckFault(uint8_t * reqBuf)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_ACK_FAULT;
}

MOTCTRL_RES MCResAckFault(uint8_t * resBuf)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_ACK_FAULT) {
    return MOTCTRL_RES_FAIL;
  }

  return (MOTCTRL_RES)resBuf[1];
}

void MCReqRetrieveIndicator(uint8_t * reqBuf, MOTCTRL_INDIID indiID)
{
  if (reqBuf == 0) {
    return;
  }
  reqBuf[0] = (uint8_t)MOTCTRL_CMD_RETRIEVE_INDICATOR;
  reqBuf[1] = (uint8_t)indiID;
}

MOTCTRL_RES MCResRetrieveIndicator(uint8_t * resBuf, MOTCTRL_INDIID * indiID, float * indiData)
{
  if (resBuf == 0 || resBuf[0] != MOTCTRL_CMD_RETRIEVE_INDICATOR) {
    return MOTCTRL_RES_FAIL;
  }
  *indiID = (MOTCTRL_INDIID)resBuf[1];

  uint8_t indiDataPtr[4];

    indiDataPtr[0] = resBuf[4];
    indiDataPtr[1] = resBuf[5];
    indiDataPtr[2] = resBuf[6];
    indiDataPtr[3] = resBuf[7];
  float * indiDataFloat = (float *)indiDataPtr;
  *indiData = *indiDataFloat;
  return (MOTCTRL_RES)resBuf[2];
}

void SendCmd2Motor(uint8_t * buf)
{

}

void ReceiveResFromMotor(uint8_t * buf)
{
  
}
