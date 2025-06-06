#ifndef CAN_NODE_H_
#define CAN_NODE_H_

#include <Arduino.h>
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

typedef unsigned char byte;
// Number of axes
#define NUM_AXES  2
// Baudrates
#define CAN_BAUDRATE  500000
#define UART_BAUDRATE 115200
// ODrive Nodes
#define JOINT1_NODE_ID 0X001
#define JOINT2_NODE_ID 0X002
#define JOINT3_NODE_ID 0X003
#define JOINT4_NODE_ID 0X004
#define JOINT5_NODE_ID 0X005
#define JOINT6_NODE_ID 0X006
// Cammand IDs
#define ODRIVE_HEARTBEAT_MESSAGE  0x001
#define ODRIVE_ESTOP_MESSAGE      0x002
#define GET_MOTOR_ERROR           0x003
#define GET_ENCODER_ERROR         0x004
#define GET_SENSORLESS_ERROR      0x005
#define SET_AXIS_NODE_ID          0x006
#define SET_AXIS_REQUESTED_STATE  0x007
#define SET_AXIS_STARTUP_CONFIG   0x008
#define GET_ENCODER_ESTIMATES     0x009
#define GET_ENCODER_COUNT         0x00A
#define SET_CONTROLLER_MODES      0x00B
#define SET_INPUT_POS             0x00C
#define SET_INPUT_VEL             0x00D
#define SET_INPUT_TORQUE          0x00E
#define SET_VELOCITY_LIMIT        0x00F
#define START_ANTICOGGING         0x010
#define SET_TRAJ_VEL_LIMIT        0x011
#define SET_TRAJ_ACCEL_LIMITS     0x012
#define SET_TRAJ_INERTIA          0x013
#define GET_IQ                    0x014
#define GET_SENSORLESS_ESTIMATES  0x015
#define REBOOT_ODRIVE             0x016
#define GET_VBUS_VOLTAGE          0x017
#define CLEAR_ERRORS              0x018
#define SET_LINEAR_COUNT          0x019
#define SET_POSITION_GAIN         0x01A
#define SET_VEL_GAINS             0x01B
// Axis state resquest values
#define AXIS_STATE_UNDEFINED                          0
#define AXIS_STATE_IDLE                               1
#define AXIS_STATE_STARTUP_SEQUENCE                   2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE          3
#define AXIS_STATE_MOTOR_CALIBRATION                  4
#define AXIS_STATE_ENCODER_INDEX_SEARCH               6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION         7
#define AXIS_STATE_CLOSED_LOOP_CONTROL                8
#define AXIS_STATE_LOCKIN_SPIN                        9
#define AXIS_STATE_ENCODER_DIR_FIND                   10
#define AXIS_STATE_HOMING                             11
#define AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION  12
#define AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION     13
// ODrive Error values
#define CONTROL_ITERATION_MISSED                      0x1
#define DC_BUS_UNDER_VOLTAGE                          0x2
#define DC_BUS_OVER_VOLTAGE                           0x4
#define DC_BUS_OVER_REGEN_CURRENT                     0x8
#define DC_BUS_OVER_CURRENT                           0x10
#define BRAKE_DEADTIME_VIOLATION                      0x20
#define BRAKE_DUTY_CYCLE_NAN                          0x40
#define INVALID_BRAKE_RESISTANCE                      0x80
// Axis Error values
#define INVALID_STATE                                 0x1
#define WATCHDOG_TIMER_EXPIRED                        0x800
#define MIN_ENDSTOP_PRESSED                           0x1000
#define MAX_ENDSTOP_PRESSED                           0x2000
#define ESTOP_REQUESTED                               0x4000
#define HOMING_WITHOUT_ENDSTOP                        0x20000
#define OVER_TEMP                                     0x40000
#define UNKNOWN_POSITION                              0x80000
// Encoder Error values
#define UNSTABLE_GAIN                                 0x1
#define CPR_POLEPAIRS_MISMATCH                        0x2
#define NO_RESPONSE                                   0x4
#define UNSUPPORTED_ENCODER_MODE                      0x8
#define ILLEGAL_HALL_STATE                            0x10
#define INDEX_NOT_FOUND_YET                           0x20
#define ABS_SPI_TIMEOUT                               0x40
#define ABS_SPI_COM_FAIL                              0x80
#define ABS_SPI_NOT_READY                             0x100
#define HALL_NOT_CALIBRATED_YET                       0x200
// Controller Error values
#define OVERSPEED                                     0x1
#define INVALID_INPUT_MODE                            0x2
// #define UNSTABLE_GAIN                                 0x4
#define INVALID_MIRROR_AXIS                           0x8
#define INVALID_LOAD_ENCODER                          0x10
#define INVALID_ESTIMATE                              0x20
#define INVALID_CIRCULAR_RANGE                        0x40
#define SPINOUT_DETECTED                              0x80

typedef float FLOAT;

typedef union {
    FLOAT f;
    char fBuff[sizeof(FLOAT)];
} dataUnion;

typedef union {
  int16_t i;
  char fBuff[sizeof(int16_t)];
} dataInt16;

typedef union {
    unsigned int i;
    char iBuff[sizeof(int)];
} int_dataUnion;

class CAN_Node {
  private:
    int node_id;
    int dir;
    int axis_error;
    int axis_current_state;
    int axis_prev_state;
  public:
    CAN_Node(byte _node_id);
    void sendCommand(byte cmd_id, byte value);
    void sendGetCommand(byte cmd_id, byte value);
    void setAxisRequestedState(byte state);
    void setInputPos(float pos, float vel);
    void setPositionGain(float _gain);
    void sendFloat(byte cmd_id, float val);
    void setTrajAccelLimits(float accel, float decel);
    int getNodeId(void);
    void setSpinDirection(int dir);
    int getSpinDirection();
    void setAxisError(int value);
    void setAxisCurrentState(int value);
    void setAxisPrevState(int value);
    int getAxisError();
    int getAxisCurrentState();
    int getAxisPrevState();
};

#define FRAME_ID(node_id, cmd_id)  node_id << 5 | cmd_id

CAN_Node::CAN_Node(byte _node_id) : node_id(_node_id), axis_prev_state(0) {}

void CAN_Node::sendCommand(byte cmd_id, byte value) {
  CAN_message_t msg;
  msg.id = FRAME_ID(node_id, cmd_id);
  msg.buf[0] = value;
  can1.write(msg);
}

void CAN_Node::sendGetCommand(byte cmd_id, byte value) {
  CAN_message_t msg;
  msg.id = FRAME_ID(node_id, cmd_id);
  msg.len = 8;
  msg.flags.remote = true;
  msg.buf[0] = value;
  can1.write(msg);
}

void CAN_Node::setAxisRequestedState(byte state) {
  CAN_message_t msg;
  msg.id = FRAME_ID(node_id, SET_AXIS_REQUESTED_STATE);
  msg.buf[0] = state;
  can1.write(msg);
}

void CAN_Node::setInputPos(float pos, float vel) {
  CAN_message_t msg;
  dataUnion position;
  dataInt16 FFvel;
  position.f = this->dir*pos;
  FFvel.i = this->dir*((int16_t) (vel*1000.0));
  msg.id = FRAME_ID(node_id, SET_INPUT_POS);
  for(int i = 0; i < sizeof(FLOAT); i++) {
    msg.buf[i] = position.fBuff[i];
  }
  for(int i = 4; i < 4 + sizeof(FFvel.i); i++) {
    msg.buf[i] = FFvel.fBuff[i];
  }
  can1.write(msg);
}

void CAN_Node::setPositionGain(float _gain) {
  CAN_message_t msg;
  dataUnion gain;
  gain.f = _gain;
  msg.id = FRAME_ID(node_id, SET_POSITION_GAIN);
  for(int i = 0; i < sizeof(FLOAT); i++) {
    msg.buf[i] = gain.fBuff[i];
  }

  can1.write(msg);
}

void CAN_Node::sendFloat(byte cmd_id, float val) {
  CAN_message_t msg;
  dataUnion value;
  value.f = val;
  msg.id = FRAME_ID(node_id, cmd_id);
  for(int i = 0; i < sizeof(FLOAT); i++) {
    msg.buf[i] = value.fBuff[i];
  }
  can1.write(msg);
}

void CAN_Node::setTrajAccelLimits(float accel, float decel) {
  CAN_message_t msg;
  dataUnion acceleration, decelleration;
  acceleration.f = accel;
  decelleration.f = decel;
  msg.id = FRAME_ID(node_id, SET_TRAJ_ACCEL_LIMITS);
  for(int i = 0; i < sizeof(FLOAT); i++)
    msg.buf[i] = acceleration.fBuff[i];
  for(int i = 0; i < sizeof(FLOAT); i++)
    msg.buf[i + 4] = decelleration.fBuff[i];
  can1.write(msg);
}

int CAN_Node::getNodeId(void) {
  return node_id;
}

void CAN_Node::setSpinDirection(int dir) {
  this->dir = dir;
}

int CAN_Node::getSpinDirection() {
  return this->dir;
}

void CAN_Node::setAxisError(int value) {
  this->axis_error = value;
}

void CAN_Node::setAxisCurrentState(int value) {
  this->axis_current_state = value;
}

void CAN_Node::setAxisPrevState(int value) {
  this->axis_prev_state = value;
}

int CAN_Node::getAxisError() {
  return this->axis_error;
}

int CAN_Node::getAxisCurrentState() {
  return this->axis_current_state;
}

int CAN_Node::getAxisPrevState() {
  return this->axis_prev_state;
}



#endif  /* CAN_NODE_H_ */
