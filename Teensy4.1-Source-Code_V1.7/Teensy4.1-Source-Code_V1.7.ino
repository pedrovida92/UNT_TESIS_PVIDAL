#include <stdint.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include "FileManager.h"
#include "Lib/commands.c"
#include "Button.h"
#include "CAN_Node.h"
#include "Robot.h"

/* Pins */
#define LED_PIN     13
#define BUTTON_PIN1 33
#define BUTTON_PIN2 34

#define NUM_JOINTS  6
#define NUM_RUTINES 3
#define NUM_POINTS  28

#define CLOCKWISE    -1
#define ANTICLOCKWISE 1

#define REDUCTION_RATIO_JOINT1   94.5   // 56/48*81
#define REDUCTION_RATIO_JOINT2  262.6   // 52/20*101
#define REDUCTION_RATIO_JOINT3  153.9   // 76/40*81
#define REDUCTION_RATIO_JOINT4   97.2   // 48/40*81
#define REDUCTION_RATIO_JOINT5   63.75  // 50/40*51
#define REDUCTION_RATIO_JOINT6   51.0

HardwareSerial *Curr_Serial;

uint32_t _offset_time = 1000;
uint32_t _prevMillis = 0;
bool _start_flag = false, _init_secuence_flag = false, _axis_state_changed = false;
#define _at_(ms)        if (_prevMillis == (ms + _offset_time + Tm*1000))

#define MAIN_RUTINE()                       \
    static unsigned int _deltaT = Tm*1000;  \
    if (millis() - _prevMillis >= _deltaT)  \
        if (_prevMillis += _deltaT)

#define SET_RUTINE(i, period)                \
    if (millis() - _prevTimes[i] >= period)  \
        if (_prevTimes[i] += period)

uint32_t _prevTimes[NUM_RUTINES];

float inf_limits_joint[] = { -46, -95, -24, -48.5, -18, -24.5};
float sup_limits_joint[] = {  46,  54,  79,  47.5,  20,  24.5};

float reduction_ratios[] = { REDUCTION_RATIO_JOINT1,
                             REDUCTION_RATIO_JOINT2,
                             REDUCTION_RATIO_JOINT3,
                             REDUCTION_RATIO_JOINT4,
                             REDUCTION_RATIO_JOINT5,
                             REDUCTION_RATIO_JOINT6 };

CAN_Node Axis[] = { JOINT1_NODE_ID,
                    JOINT2_NODE_ID,
                    JOINT3_NODE_ID,
                    JOINT4_NODE_ID,
                    JOINT5_NODE_ID,
                    JOINT6_NODE_ID };

int current_axis = -1;
int aux_current_axis = -1;
uint8_t current_joint = 0;

Button btn1, btn2;

int counts = 0;
int varFlag = 0;

double traj_time = 1.5;
double traj_pv = 0.75;
double traj_pa = 0.75;
char traj_ip = 'h';
bool traj_enable = true;

float Qencoder[6];
float dQencoder[6];
bool encoderStimatesFinished = false;
bool waiting_for_move = false;
pose P, Pa;
art Qenc;
mth Tc;

art mttoJoint = {0, -30, -45, 0, 75, 0};
art transJoint = {0, 40, -56, 0, 0, 0};

// -------------------------------------
bool catch_point_flag = false;
String point_name;

typedef struct {
    String tag;
	art Q;
} point;

point points[28];
int pos_pointer = 0;
// -------------------------------------


byte verb = true;
byte joint_feedback_flag = true;
byte pos_feedback_flag = true;
byte _feedback_flag = true;
int mode = 0, init_secuence_step = 0;

bool read_messages = false;
bool send_robot_heartbeat = false;

String strTraj = "";
// char strTraj[100] = "";
File trajFile;

pose P1, P2, P3, P4;

typedef struct {
  String cmd;
  float Pa[3], Pf[6];
  float T, pv, t0;
  char ip;
} trajectory;

trajectory trajs[100];
int num_trajs = 0;
int traj_counter = 0;
int num_exe = -1;
int exe_counter = 0;
bool exeflag = false;

float exe_factor = 1.0;
float limit_motor_vel = 50.0;

int cont = 0;

int_dataUnion axis_error, axis_current_state;

#define fanucProgram_1()                \
  _at_(0000)    _PTP(P1, 1.0, pv, '1'); \
  _at_(1000)    _PTP(P2, 1.0, pv, '1'); \
  _at_(2000)    _PTP(P3, 1.0, pv, '1'); \
  _at_(3000)    _LIN(P4, 1.0, pv, '1'); \
  _at_(4000)    _HOME();

void initial_secuence() {
    
}

// For HC-05 Bluetooth Module
#define EN  32
#define VCC 33

void Init_Bluetooth() {
    pinMode(VCC, OUTPUT);
    pinMode(EN, OUTPUT);
    digitalWrite(VCC, HIGH);
    digitalWrite (EN, LOW);
    delay (500);
    digitalWrite(EN, HIGH);
    Serial8.begin(115200);
    while (!Serial8);
    Serial.println("Bluetooth encendido");
}

long long tiempo_0;

void setup(){
    Serial.begin(1E6);
    while (!Serial);  // Esperar a que se inicie comunicacion serial

    // Iniciar comunicacion CAN
    can1.begin();
    can1.setBaudRate(1E6);

    // see if the card is present and can be initialized:
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("Card failed, or not present");
    //   while (1) {
    //     // No SD card, so don't do anything more - stay stuck here
    //   }
    }

    pinMode(LED_PIN, OUTPUT);
    button_init(&btn1, {BUTTON_PIN1, 1, 50, 500});
    button_init(&btn2, {BUTTON_PIN2, 1, 50, 500, INPUT_PULLUP});

    Axis[0].setSpinDirection(ANTICLOCKWISE);
    Axis[1].setSpinDirection(CLOCKWISE);
    Axis[2].setSpinDirection(ANTICLOCKWISE);
    Axis[3].setSpinDirection(ANTICLOCKWISE);
    Axis[4].setSpinDirection(ANTICLOCKWISE);
    Axis[5].setSpinDirection(ANTICLOCKWISE);
  
  	pv = 0.8;
  	// P1 = {400, 200, 300, 0, -90, 180};
  	// P2 = {400, 200, 600, 0,-180, 180};
  	// P3 = {400,-300, 200, 0, -90, 180};
  	// P4 = {400,-300, 600, 0,-180, 180};
    P1 = {300, 160, 210, 0, -90, 180};
    P2 = {300, 160, 500, 0,-180, 180};
    P3 = {300,-210, 160, 0, -90, 180};
    P4 = {300,-210, 500, 0,-180, 180};

    FileManager.setVerboseMode(verb);

    // Init_Bluetooth();

    requestEncoderEstimates();

    tiempo_0 = millis();
}

void requestEncoderEstimates() {
    encoderStimatesFinished = false;
    for (int i = 0; i < NUM_JOINTS; i++)
        Axis[i].sendGetCommand(GET_ENCODER_ESTIMATES, 0);
}

bool command_rdy = false;
String command = "";
String curr_dir = "";
// String curr_file = "";

long long t0 = 0, t1 = 0;

void loop(){
    // button_handle(&btn1, &counterUp);
    // button_handle(&btn2, &counterDown);
    MAIN_RUTINE() {
        static bool bloqueo = false;
        if (!bloqueo) {
            if (((millis() - tiempo_0) > 1000)) {
                Qc = arr2art(Qencoder);
                bloqueo = true;
            }
        }
        t0 = micros();
        // fanucProgram_3();
        if (_start_flag) {
          _at_((int)(0*traj_time*1000))   _PTP({0,175,325, 90,0,180}, traj_time, traj_pv, traj_ip);
          _at_((int)(1*traj_time*1000))   _CIRC({0,475,325, 90,0,180}, {-150,325,325, 90,0,180}, traj_time, traj_pv, traj_ip);
          _at_((int)(2*traj_time*1000))   _HOME(traj_time);
          _at_((int)(3*traj_time*1000))   _start_flag = false;
        }
        
        if (exeflag) {
            trajectory curr_traj;
            if (traj_counter < num_trajs) {
                long long exeTime;
                curr_traj = trajs[traj_counter];
                exeTime = (!traj_counter) ? 0 : (int)(curr_traj.t0*1000);
                pose Pa = arr2pose(curr_traj.Pa);
                pose Pf = arr2pose(curr_traj.Pf);
                _at_ (exeTime + 100) {
                    // requestEncoderEstimates();
                    if (curr_traj.cmd == "PTP")
                    _PTP(Pf, curr_traj.T, curr_traj.pv, curr_traj.ip);
                    if (curr_traj.cmd == "LIN")
                    _LIN(Pf, curr_traj.T, curr_traj.pv, curr_traj.ip);
                    if (curr_traj.cmd == "CIRC")
                    _CIRC(Pa, Pf, curr_traj.T, curr_traj.pv, curr_traj.ip);
                    if (curr_traj.cmd == "HOME")
                    _HOME(curr_traj.T);
                    traj_counter++;
                }
            } else {
                if (trajFlags == TRAJ_NONE_FLAG) {
                    exe_counter++;
                    traj_counter = 0;
                    _offset_time = _prevMillis = millis();
                }
                if (exe_counter == num_exe) {
                    num_trajs = 0;
                    exeflag = false;
                }
            }
        }

        if (!_init_secuence_flag) {
            current_axis = aux_current_axis;
        }

        switch (trajFlags) {
            case TRAJ_PTP_FLAG: /* if (encoderStimatesFinished) */ _PTPMotionHandler();  break;
            case TRAJ_LIN_FLAG: /* if (encoderStimatesFinished) */ _LINMotionHandler();  break;
            case TRAJ_CIRC_FLAG: /* if (encoderStimatesFinished) */ _CIRCMotionHandler();  break;
            default:  break;
        }
        
        if (trajFlags != TRAJ_NONE_FLAG) {
            switch (mode) {
                case 0: _commandJoints(Qc); break;
                case 1: commandJoints();    break;
                default:    break;
            }
        }

        if (trajFlags == TRAJ_NONE_FLAG && samples) {
            // Serial.printf("Trajectory Finished!!! at %u\n", millis() - _offset_time);
        }

        static char prevTrajFlags;
        if (trajFlags == TRAJ_NONE_FLAG && prevTrajFlags != TRAJ_NONE_FLAG) {
            if (verb) {
                // Serial.println("Trajectory Finished");
                // for (int i = 0; i < 5; i++) {
                //     vec result = subV(medias[i], posiciones[i]);
                //     double r = norm(result);
                //     Serial.printf("Presicion AP%d:\t%.4f", 5-i, result);
                // }
            }
        }
        
        prevTrajFlags = trajFlags;
        // Serial.print(Axis[current_axis].getNodeId(), HEX);
        // Serial.print("\t");
        // Serial.print(Axis[current_axis].getAxisError(), HEX);
        // Serial.print("\t");
        // Serial.print(Axis[current_axis].getAxisCurrentState(),HEX);
        // Serial.println();
        t1 = micros();
        // Serial.println(t1 - t0);
    }

    if (Serial8.available()) {
        Curr_Serial = &Serial8;
        bluetoothCommandHandler();
    }

    if (Serial.available()) {
        serialCommandHandler();
    }

    // if (read_messages) {
    //   readMessagesFromODrive();
    // }

    SET_RUTINE(0, 1) {
        readMessagesFromODrive();
    }

    SET_RUTINE(1, 1000) {
        if (send_robot_heartbeat) {
            Serial.print(trajFlags);
            for (int i = 0; i < NUM_JOINTS; i++) {
                Serial.print("0x"); Serial.print(Axis[i].getAxisError(), HEX);  Serial.print("\t");
                Serial.print("0x"); Serial.print(Axis[i].getAxisCurrentState(), HEX);
                Serial.println();
            }
        }
    }

    SET_RUTINE(2, 1) {
        if (printflag) {
            Serial.printf("Point printed:  %f, %f, %f, %f, %f, %f\n",
                Qc.q1*RAD_TO_DEG,
                Qc.q2*RAD_TO_DEG,
                Qc.q3*RAD_TO_DEG,
                Qc.q4*RAD_TO_DEG,
                Qc.q5*RAD_TO_DEG,
                Qc.q6*RAD_TO_DEG);
            printflag = false;
        }
    }
}

pose arr2pose(float * arr) {
    return (pose) { arr[0], arr[1], arr[2], arr[3], arr[4], arr[5] };
}

art arr2art(float * arr) {
    return (art) { arr[0], arr[1], arr[2], arr[3], arr[4], arr[5] };
}

void pose2arr(float * arr, pose X) {
    arr[0] = (float) X.p.x;
    arr[1] = (float) X.p.y;
    arr[2] = (float) X.p.z;
    arr[3] = (float) X.o.x;
    arr[4] = (float) X.o.y;
    arr[5] = (float) X.o.z;
}

void art2arr(float * arr, art Q) {
    arr[0] = (float) Q.q1;
    arr[1] = (float) Q.q2;
    arr[2] = (float) Q.q3;
    arr[3] = (float) Q.q4;
    arr[4] = (float) Q.q5;
    arr[5] = (float) Q.q6;
}

art pose2art(pose P) {
    return (art) {P.p.x, P.p.y, P.p.z, P.o.x, P.o.y, P.o.z};
}

pose art2pose(art Q) {
    return (pose) {Q.q1, Q.q2, Q.q3, Q.q4, Q.q5, Q.q6};
}

void commandJoints() {
    if (joint_feedback_flag) {
        Serial.print(ddQc.q1*180/M_PI);  Serial.print(",");
        Serial.print(ddQc.q2*180/M_PI);  Serial.print(",");
        Serial.print(ddQc.q3*180/M_PI);  Serial.print(",");
        Serial.print(ddQc.q4*180/M_PI);  Serial.print(",");
        Serial.print(ddQc.q5*180/M_PI);  Serial.print(",");
        Serial.print(ddQc.q6*180/M_PI);  Serial.print('\n');
    }
    if (traj_enable) {
        // Axis[0].sendFloat(SET_TRAJ_VEL_LIMIT, dQc.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1);/*  delayMicroseconds(100); */
        // Axis[1].sendFloat(SET_TRAJ_VEL_LIMIT, dQc.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2);/*  delayMicroseconds(100); */
        // Axis[2].sendFloat(SET_TRAJ_VEL_LIMIT, dQc.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3);/*  delayMicroseconds(100); */
        // Axis[3].sendFloat(SET_TRAJ_VEL_LIMIT, dQc.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4);/*  delayMicroseconds(100); */
        // Axis[4].sendFloat(SET_TRAJ_VEL_LIMIT, dQc.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5);/*  delayMicroseconds(100); */
        // Axis[5].sendFloat(SET_TRAJ_VEL_LIMIT, dQc.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6);/*  delayMicroseconds(100); */
        // Axis[0].setTrajAccelLimits(ddQc.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1, ddQc.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1);/*  delayMicroseconds(100); */
        // Axis[1].setTrajAccelLimits(ddQc.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2, ddQc.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2);/*  delayMicroseconds(100); */
        // Axis[2].setTrajAccelLimits(ddQc.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3, ddQc.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3);/*  delayMicroseconds(100); */
        // Axis[3].setTrajAccelLimits(ddQc.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4, ddQc.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4);/*  delayMicroseconds(100); */
        // Axis[4].setTrajAccelLimits(ddQc.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5, ddQc.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5);/*  delayMicroseconds(100); */
        // Axis[5].setTrajAccelLimits(ddQc.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6, ddQc.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6);/*  delayMicroseconds(100); */
        // Axis[0].setInputPos(constrain(Qc.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1, inf_limits_joint[0], sup_limits_joint[0]));
        // Axis[1].setInputPos(constrain(Qc.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2, inf_limits_joint[1], sup_limits_joint[1]));
        // Axis[2].setInputPos(constrain(Qc.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3, inf_limits_joint[2], sup_limits_joint[2]));
        // Axis[3].setInputPos(constrain(Qc.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4, inf_limits_joint[3], sup_limits_joint[3]));
        // Axis[4].setInputPos(constrain(Qc.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5, inf_limits_joint[4], sup_limits_joint[4]));
        // Axis[5].setInputPos(constrain(Qc.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6, inf_limits_joint[5], sup_limits_joint[5]));
    }
    Qc_ant = Qc;
    dQc_ant = dQc;
}

void _commandJoints(art _Q) {
    // long long t0 = micros();
    // tauc = Dynamics(Qc, dQc, ddQc, (vec){0,0,0}, (vec){0,0,0});
    // long long t1 = micros();
    if (joint_feedback_flag) {
        // Serial.printf("%f, %f, %f, %f, %f, %f",
        Serial.printf("_J%f, %f, %f, %f, %f, %f\n",
            _Q.q1*RAD_TO_DEG,
            _Q.q2*RAD_TO_DEG,
            _Q.q3*RAD_TO_DEG,
            _Q.q4*RAD_TO_DEG,
            _Q.q5*RAD_TO_DEG,
            _Q.q6*RAD_TO_DEG);
        // pose X = DK(_Q, 0, 0, 0);
        // Serial.printf("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
        //     X.p.x,
        //     X.p.y,
        //     X.p.z,
        //     X.o.x*RAD_TO_DEG,
        //     X.o.y*RAD_TO_DEG,
        //     X.o.z*RAD_TO_DEG);
        // pose X1 = DK(arr2art(Qencoder), 0, 0, 0);
        // Serial.printf(",%.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
        //     X1.p.x,
        //     X1.p.y,
        //     X1.p.z,
        //     X1.o.x*RAD_TO_DEG,
        //     X1.o.y*RAD_TO_DEG,
        //     X1.o.z*RAD_TO_DEG);
        // Serial.printf("_Q%f, %f, %f, %f, %f, %f\n",
        // Serial.printf(",%f, %f, %f, %f, %f, %f\n",
        //     Qencoder[0]*RAD_TO_DEG,
        //     Qencoder[1]*RAD_TO_DEG,
        //     Qencoder[2]*RAD_TO_DEG,
        //     Qencoder[3]*RAD_TO_DEG,
        //     Qencoder[4]*RAD_TO_DEG,
        //     Qencoder[5]*RAD_TO_DEG);
        // Serial.print(tauc.q1*1e-9);  Serial.print(",");
        // Serial.print(tauc.q2*1e-9);  Serial.print(",");
        // Serial.print(tauc.q3*1e-9);  Serial.print(",");
        // Serial.print(tauc.q4*1e-9);  Serial.print(",");
        // Serial.print(tauc.q5*1e-9);  Serial.print(",");
        // Serial.print(tauc.q6*1e-9);  Serial.print('\n');
        // Serial.println(t1-t0);
    }
    if (traj_enable) {
        Axis[0].setInputPos(constrain(_Q.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1, inf_limits_joint[0], sup_limits_joint[0]),
                                        /* dQc.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1 */0);
        Axis[1].setInputPos(constrain(_Q.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2, inf_limits_joint[1], sup_limits_joint[1]),
                                        /* dQc.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2 */0);
        Axis[2].setInputPos(constrain(_Q.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3, inf_limits_joint[2], sup_limits_joint[2]),
                                        /* dQc.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3 */0);
        Axis[3].setInputPos(constrain(_Q.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4, inf_limits_joint[3], sup_limits_joint[3]),
                                        /* dQc.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4 */0);
        Axis[4].setInputPos(constrain(_Q.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5, inf_limits_joint[4], sup_limits_joint[4]),
                                        /* dQc.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5 */0);
        Axis[5].setInputPos(constrain(_Q.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6, inf_limits_joint[5], sup_limits_joint[5]),
                                        /* dQc.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6 */0);
        requestEncoderEstimates();
    }
}

void _sendMTH(art _Q) {
    mth T = dirKinematics(_Q);
    Serial.print("_T");
    Serial.print(T.R.r11);  Serial.print(",");
    Serial.print(T.R.r12);  Serial.print(",");
    Serial.print(T.R.r13);  Serial.print(",");
    Serial.print(T.R.r21);  Serial.print(",");
    Serial.print(T.R.r22);  Serial.print(",");
    Serial.print(T.R.r23);  Serial.print(",");
    Serial.print(T.R.r31);  Serial.print(",");
    Serial.print(T.R.r32);  Serial.print(",");
    Serial.print(T.R.r33);  Serial.print(",");
    Serial.print(T.p.x);    Serial.print(",");
    Serial.print(T.p.y);    Serial.print(",");
    Serial.print(T.p.z);    Serial.print('\n');
}

String Serial_readStringBefore(char ch) {
    String str = "";
    char _c;
    while ((_c = Serial.peek()) == ' ')
        Serial.read(); // Ignore blanks
    while ((_c = Serial.peek()) != ch) {
        str += (char) _c;
        _c = Serial.read();
    }
    return str;
}

void serialCommandHandler() {
    // String command = "";
    char ch = Serial.peek();
    while ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')) {
        command += (char) ch;   // Concatenate char
        Serial.read();          // Consume char
        ch = Serial.peek();     // Peek next char
    }
    wordCommand();
    oneLetterCommand();
    while (Serial.read() != '\n');
    command = "";
}

void bluetoothCommandHandler() {
    static bool command_ready = false;
    char ch = Curr_Serial->read();
    Curr_Serial->print(ch);
    // Serial.println(command_ready);
    if (((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')) && !command_ready)
        command += (char) ch;   // Concatenate char
    if (ch == ' ') {
        Curr_Serial->println((char)Curr_Serial->peek());
        command_ready = true;
        // Serial.println("Espacio");
    }
    // Serial.println(command);
    if (ch == '\n' || ch == '\r') {
        command_ready = false;
        command = "";
        // Serial.println("Fin de linea");
    }

    if (command_ready)  {
        // wordCommand(*Curr_Serial);
    }
}

void wordCommand() {
    if (command == "VRB") {
        if (verb ^= true) Serial.printf("VERBOSE mode!!!\n");
        FileManager.setVerboseMode(verb);
    }
    if (command == "SND") {
        joint_feedback_flag ^= true;
    }
    if (command == "MODE") {
        mode = Serial.parseInt();
        if (mode < 0 || mode > 2) {
            mode = 0;
        }
        if (verb) Serial.printf("Traj. Mode:\t%d\n", mode);
    }
    if (command == "INIT") {
        if (verb) Serial.println("Traj. Init ...");
        Axis[current_axis].sendCommand(CLEAR_ERRORS, 0);
        if (verb) Serial.println("Error Cleared");
        delay(500);
        // Axis[current_axis].setAxisRequestedState(AXIS_STATE_IDLE);
        // if (verb) Serial.println("Axis state Idle");
        // delay(500);
        _init_secuence_flag = true;
        _axis_state_changed = true;
        init_secuence_step = 0;
        cont = 0;
    }
    if (command == "TJE") {
        traj_enable = Serial.parseInt();
        if (verb) Serial.printf("Traj. enable:\t%s\n",
                    traj_enable ? "TRUE" : "FALSE");
    }
    if (command == "TIK") {
        traj_time = Serial.parseFloat();
        if (traj_time == 0.0) traj_time = 1;
        if (verb) Serial.printf("Traj. time:\t%.4lf\n", traj_time);
    }
    if (command == "PV") {
        traj_pv = Serial.parseFloat();
        if (verb) Serial.printf("Velocity[%]:\t%.4lf\n", traj_pv);
    }
    if (command == "PA") {
        traj_pv = Serial.parseFloat();
        if (verb) Serial.printf("Velocity[%]:\t%.4lf\n", traj_pa);
    }
    if (command == "IP") {
        traj_ip = Serial.read();
        traj_ip = Serial.read();
        char* ipName = "";
        if (traj_ip == 'c') {
            ipName = "Cúbico";
        } else if (traj_ip == 'q') {
            ipName = "Quíntico";
        } else if (traj_ip == '1') {
            ipName = "Curva S";
        } else if (traj_ip == '2') {
            ipName = "Curva S2";
        } else {
            ipName = "Harmonic S curve";
        }
        if (verb) Serial.printf("Interp. type:\t%s\n", ipName);
    }
    if (command == "WAIT") {
        float time = Serial.parseFloat();
        _WAIT(time);
    }
    if (command == "PTP" || command == "LIN" || command == "CIRC" || command == "HOME" ||
        command == "POSE" || command == "JOINT" || command == "DPOSE" || command == "DJOINT" ||
        command == "MOVE" || command == "MAINTENANCE" || command == "TRANSPORT") {
        double xa, ya, za, x, y, z, a, b, c;
        // art Qc = 
        strTraj = "";
        strTraj.concat(command); strTraj.concat("\t");
        if (command != "HOME" && command != "MAINTENANCE" && command != "TRANSPORT") {
            if (command == "CIRC") {
                xa = Serial.parseFloat();
                ya = Serial.parseFloat();
                za = Serial.parseFloat();
                strTraj.concat(xa); strTraj.concat(",\t");
                strTraj.concat(ya); strTraj.concat(",\t");
                strTraj.concat(za); strTraj.concat(",\t");
            }
            x = Serial.parseFloat();
            y = Serial.parseFloat();
            z = Serial.parseFloat();
            a = Serial.parseFloat();
            b = Serial.parseFloat();
            c = Serial.parseFloat();
            strTraj.concat(x); strTraj.concat(",\t");
            strTraj.concat(y); strTraj.concat(",\t");
            strTraj.concat(z); strTraj.concat(",\t");
            strTraj.concat(a); strTraj.concat(",\t");
            strTraj.concat(b); strTraj.concat(",\t");
            strTraj.concat(c); strTraj.concat(",\t");
        }
        strTraj.concat(traj_time);strTraj.concat(",\t");
        strTraj.concat(traj_pv);strTraj.concat(",\t");
        strTraj.concat(traj_ip);
        P = {{x, y, z}, {a, b, c}};
        Pa = {{xa, ya, za}, {a, b, c}};
        if (command == "PTP") {
            _PTP(P, traj_time, traj_pv, traj_ip);
        } else if (command == "LIN") {
            _LIN(P, traj_time, traj_pv, traj_ip);
        } else if (command == "CIRC") {
            _CIRC(Pa, P, traj_time, traj_pv, traj_ip);
        } else if (command == "HOME") {
            _HOME(traj_time);
        } else if (command == "MAINTENANCE") {
            _MOVE(dotART(DEG_TO_RAD, mttoJoint), traj_time, traj_pv, traj_ip);
        } else if (command == "TRANSPORT") {
            _MOVE(dotART(DEG_TO_RAD, transJoint), traj_time, traj_pv, traj_ip);
        } else if (command == "MOVE") {
            _MOVE(dotART(DEG_TO_RAD, pose2art(P)), traj_time, traj_pv, traj_ip);
        } else if (command == "POSE" || command == "JOINT" ||
                   command == "DPOSE" || command == "DJOINT") {
            strTraj = "";
            float arrQ[6], arrQc[6], max = 0, diff, speed = 5;
            int ind = 0;
            art Q;
            if (command == "POSE")  Q = ikSolve((pose) {P.p, deg2rad(P.o)}, Qc);
            if (command == "DPOSE")  Q = ikSolve((pose) {P.p, deg2rad(P.o)}, Qc);
            if (command == "JOINT")  Q = dotART(DEG_TO_RAD, (art) {P.p.x, P.p.y, P.p.z, P.o.x, P.o.y, P.o.z});
            if (command == "DJOINT")    Q = addART(Qc, dotART(DEG_TO_RAD, (art){x,y,z,a,b,c}));
            art2arr(arrQ, Q);
            art2arr(arrQc, Qc);
            for (int i = 0; i < 6; i++) {
                diff = (arrQ[i] - arrQc[i]);
                diff = (diff < 0.0) ? -diff: diff;
                if (diff > max) {
                    max = diff;
                    ind = i;
                }
            }
            double aux_time = max*reduction_ratios[ind]/(speed*TWO_PI);
            Serial.println(aux_time);
            _MOVE(Q, aux_time, 0.75, 'q');
        }
        if (verb) Serial.println(strTraj);
    }
    if (command == "FPTP") {
        String tag = Serial_readStringBefore('\n');
        Serial.println(tag);
        art Q;
        for (int i = 0; i < NUM_POINTS; i++) {
            if (points[i].tag == tag) {
                Q = points[i].Q;
                i = NUM_POINTS;
            }
        }
        _MOVE(dotART(DEG_TO_RAD, Q), traj_time, traj_pv, traj_ip);
    }
    if (command == "SHW") {
        String filename = FileManager.getCurrentFile() + ".txt";
        const char * path = filename.c_str();
        if (SD.exists(path)) {
            FileManager.readFile(path);
        } else {
            Serial.printf("%s does not exist\n", filename.c_str());
        }
    }
    if (command == "SVD") {
        String _path = "/" + FileManager.getCurrentFile() + ".txt";
        FileManager.appendFile(_path, strTraj + "\n");
        strTraj = "";
    }
    if (command == "SEL") {
        String curr_file = Serial_readStringBefore('\n');
        FileManager.setCurrentFile(curr_file);
        if (SD.exists((curr_file + ".txt").c_str())) {
            if (verb) Serial.printf("Current file:\t%s\n", ("/" + curr_file + ".txt").c_str());
        } else {
            if (verb) Serial.printf("%s does not exist\n", ("/" + curr_file + ".txt").c_str());
        }
    }
    if (command == "CLR") {
        String _path = "/" + FileManager.getCurrentFile() + ".txt";
        FileManager.writeFile(_path, "");
    }
    if (command == "DLT") {
        char path_arr[PATH_SIZE];
        String _path = "/" + FileManager.getCurrentFile() + ".txt";
        _path.toCharArray(path_arr, PATH_SIZE);
        trajFile = SD.open(path_arr, FILE_READ);
        int num_lines = 0;
        String temp;
        while (trajFile.available()) {
            String line = trajFile.readStringUntil('\n');
            if (trajFile.available()) {
            temp.concat(line);temp.concat("\n");
            }
        }
        trajFile.close();
        SD.remove(path_arr);
        trajFile = SD.open(path_arr, FILE_WRITE);
        trajFile.print(temp);
        trajFile.close();
        Serial.println("Line deleted");
    }
    if (command == "EXE") {
        // Serial.println("In EXE Command");
        num_exe = (int)Serial.parseFloat();
        String filename = FileManager.getCurrentFile() + ".txt";
        const char * path = filename.c_str();
        exeflag = ((num_exe > 0) && (SD.exists(path)));
        Serial.println(num_exe);
        Serial.println(exeflag);
        if (exeflag) {
            exe_counter = 0;
            String _path = "/" + FileManager.getCurrentFile() + ".txt";
            trajFile = SD.open(_path.c_str(), FILE_READ);
            num_trajs = 0;
            trajectory aux_traj;
            while (trajFile.available()) {
                // Serial.println("In While Loop");
                _offset_time = _prevMillis = millis();
                aux_traj.cmd = trajFile.readStringUntil('\t');
                if (aux_traj.cmd == "CIRC") for (int i = 0; i < 3; i++) aux_traj.Pa[i] = trajFile.parseFloat();
                if (aux_traj.cmd != "HOME") for (int i = 0; i < 6; i++) aux_traj.Pf[i] = trajFile.parseFloat();
                aux_traj.T = trajFile.parseFloat()*exe_factor;
                aux_traj.pv = trajFile.parseFloat();
                String ip = trajFile.readStringUntil('\n');
                aux_traj.t0 = (!num_trajs) ? 0.0 : trajs[num_trajs-1].t0 + trajs[num_trajs-1].T;
                trajs[num_trajs++] = aux_traj;
            }
        }
    }
    if (command == "EXEVEL") {
        float prev_exe_factor = exe_factor;
        exe_factor = Serial.parseFloat();
        if (!exe_factor) exe_factor = prev_exe_factor;
        if (verb) {
            Serial.print("Execution Velocity:\ttraj_time * ");
            Serial.println(exe_factor);
        }
    }
    if (command == "SEXEVEL") {
        String _path = "/" + FileManager.getCurrentFile() + ".txt";
        const char * path_arr = _path.c_str();
        trajFile = SD.open(path_arr, FILE_READ);
        int num_lines = 0;
        String temp;
        while (trajFile.available()) {
            String line = trajFile.readStringUntil('\n');
            char * line_arr = (char *)line.c_str();
            String cmd = readUntil(line_arr, '\t');
            temp += cmd + '\t';
            if (cmd == "CIRC")   for (int i = 0; i < 3; i++)
                temp += readUntil(line_arr, '\t') + '\t';  // Pa[1:3]
            if (cmd != "HOME")  for (int i = 0; i < 6; i++)
                temp += readUntil(line_arr, '\t') + '\t';  // Pf[1:6]
            temp += readUntil(line_arr, '\t').toFloat()*exe_factor; // T
            temp += ",\t";
            temp += readUntil(line_arr, '\t') + '\t'; // pv
            temp += readUntil(line_arr, '\0'); // ip
        }
        trajFile.close();
        SD.remove(path_arr);
        trajFile = SD.open(path_arr, FILE_WRITE);
        trajFile.print(temp);
        trajFile.close();
        Serial.println("Program modified");
        exe_factor = 1.0;
        if (verb) {
            Serial.print("Execution Velocity:\ttraj_time * ");
            Serial.println(exe_factor);
        }
    }
    if (command == "HELP") {
        int option = Serial.parseInt();
        if (option == 1) {
            if (SD.exists("Help1.txt"))
            FileManager.readFile("Help1.txt");
            else
            Serial.printf("Help1.txt does not exist\n");
        } else if (option == 2) {
            if (SD.exists("Help2.txt"))
            FileManager.readFile("Help2.txt");
            else
            Serial.printf("Help2.txt does not exist\n");
        } else {
            Serial.println("Existen dos archivos HELP:\nIngrese nuevamente el comando con un argumento entero");
        }
    }
    if (command == "LIMITVEL") {
        limit_motor_vel = Serial.parseFloat();
    }
    if (command == "TCP") {
        double x, y, z, a, b, c;
        x = Serial.parseFloat();
        y = Serial.parseFloat();
        z = Serial.parseFloat();
        a = Serial.parseFloat();
        b = Serial.parseFloat();
        c = Serial.parseFloat();
        TCP.p = (vec){x, y, z};
        TCP.R = rpy2mat(deg2rad((vec){a, b, c}));
        if (verb) Serial.println("TCP changed");
        if (verb) printlnMTH(TCP);
    }
    if (command == "LS") {
        if (verb) {
            Serial.println("Listing directories:");
            FileManager.listDir(SD.open("/"), 0);
        }
    }
    if (command == "MKDIR" || command == "CRT" || command == "RM" || command == "RD" ||
        command == "RNM" || command == "DEL") {
        char path_arr[PATH_SIZE], prev_path_arr[PATH_SIZE];
        String name = "", prev_name = "", _path;
        // -------------- Catch Directory Name --------------
        char _ch;
        if (command == "RNM") {
            while ((_ch = Serial.peek()) == ' ')
                Serial.read(); // Ignore blanks
            while ((_ch = Serial.peek()) != ' ') {
                prev_name += (char) _ch;
                _ch = Serial.read();
            }
            prev_name.trim();
            _path = curr_dir + '/' + prev_name + ".txt";
            _path.toCharArray(prev_path_arr, PATH_SIZE);
        }
        while ((_ch = Serial.peek()) != '\n') {
            name += (char) _ch;
            _ch = Serial.read();
        }
        name.trim(); // Delete Blanks at start and end
        // --------------------------------------------------
        if (command == "CRT" || command == "RM" || command == "RD" || command == "RNM")
            name += ".txt";
        String path = curr_dir + '/' + name;
        path.toCharArray(path_arr, PATH_SIZE);
        if (command == "MKDIR")
            FileManager.createDir(path_arr);
        if (command == "CRT") {
            if (path == "/Help.txt") Serial.println("Help.txt is a protected file. You can not create a file with that name");
            else FileManager.createFile(path_arr);
        }
        if (command == "RM")
            if (path == "/Help.txt") Serial.println("Help.txt is a protected file. You can not remove a file with that name");
            else FileManager.deleteFile(path_arr);
        if (command == "RD")
            FileManager.readFile(path_arr);
        if (command == "RNM") {
            if (_path == "/Help.txt" || path == "/Help.txt")
            Serial.println("Help.txt is a protected file. You can not rename a file with that name");
            else FileManager.renameFile(prev_path_arr, path_arr);
        }
    }
    if (command == "REQJ") {
        Serial.print("_J");
        Serial.print(Qc.q1*180/M_PI);    Serial.print(",");
        Serial.print(Qc.q2*180/M_PI);    Serial.print(",");
        Serial.print(Qc.q3*180/M_PI);    Serial.print(",");
        Serial.print(Qc.q4*180/M_PI);    Serial.print(",");
        Serial.print(Qc.q5*180/M_PI);    Serial.print(",");
        Serial.println(Qc.q6*180/M_PI);
    }
    if (command == "STOP") {
        trajFlags = TRAJ_NONE_FLAG;
    }
    if (command == "CATCH") {
        point_name = Serial_readStringBefore('\n');
        Serial.println(point_name);
        catch_point_flag = true;
        requestEncoderEstimates();
    }
    if (command == "POINTS") {
        for (int i = 0; i < NUM_POINTS; i++) {
            if (points[i].tag != "") {
                Serial.printf("%s, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
                        points[i].tag.c_str(), points[i].Q.q1, points[i].Q.q2, points[i].Q.q3,
                        points[i].Q.q4, points[i].Q.q5, points[i].Q.q6);
            }
        }
    }
    if (command == "POSGAIN") {
        float gain = Serial.parseFloat();
        if (verb)   Serial.printf("Position gain:\t%.4f\n", gain);
        Axis[current_axis].setPositionGain(gain);
    }
    if (command == "DYNAMICS") {
        double q1, q2, q3, q4, q5, q6;
        double qp1, qp2, qp3, qp4, qp5, qp6;
        double qpp1, qpp2, qpp3, qpp4, qpp5, qpp6;
        q1 = Serial.parseFloat();
        q2 = Serial.parseFloat();
        q3 = Serial.parseFloat();
        q4 = Serial.parseFloat();
        q5 = Serial.parseFloat();
        q6 = Serial.parseFloat();
        qp1 = Serial.parseFloat();
        qp2 = Serial.parseFloat();
        qp3 = Serial.parseFloat();
        qp4 = Serial.parseFloat();
        qp5 = Serial.parseFloat();
        qp6 = Serial.parseFloat();
        qpp1 = Serial.parseFloat();
        qpp2 = Serial.parseFloat();
        qpp3 = Serial.parseFloat();
        qpp4 = Serial.parseFloat();
        qpp5 = Serial.parseFloat();
        qpp6 = Serial.parseFloat();
        long long t0 = micros();
        art tau = Dynamics((art){q1, q2, q3, q4, q5, q6}, (art){qp1, qp2, qp3, qp4, qp5, qp6}, (art){qpp1, qpp2, qpp3, qpp4, qpp5, qpp6}, (vec){0,0,0}, (vec){0,0,0});
        long long t1 = micros();
        Serial.print(tau.q1*1e-9);Serial.print(", ");
        Serial.print(tau.q2*1e-9);Serial.print(", ");
        Serial.print(tau.q3*1e-9);Serial.print(", ");
        Serial.print(tau.q4*1e-9);Serial.print(", ");
        Serial.print(tau.q5*1e-9);Serial.print(", ");
        Serial.print(tau.q6*1e-9);Serial.print(", ");
        Serial.println();
        Serial.println(t1 - t0);
    }
}

String readUntil(char * &line, char ch) {
    String str;
    char c;
    while ((c = *line++) != ch)
        str += c;
    return str;
}

void changeTimeTraj(String &line, float time_traj) {
    // char * str = (char*)line.c_str();
    // int cont = 0;
    // String cmd;
    // while (*str++) {
    //     if (!cont)  cmd += *str;
    //     if (*str == '\t') {
    //         cont++;
    //     }
    //     if ()
    // }
}

void oneLetterCommand() {
    if (command == "s") {
        _start_flag = !_start_flag;
        _offset_time = _prevMillis = millis();
    }
    if (command == "j") {
        aux_current_axis = Serial.parseInt() - 1;
        if (aux_current_axis < 0 || aux_current_axis > (NUM_JOINTS - 1)) {
            aux_current_axis = -1;
            if (verb)   Serial.println("Joint out of range");
        } else {
            if (verb)   Serial.printf("Joint : %d\n", aux_current_axis + 1);
        }
    }
    if (command == "f") {
        if (verb)   Serial.print("Requesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE in Joint ");
        if (verb)   Serial.println(current_axis + 1);
        // printf("Requesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE in Joint %d\n", current_axis + 1);
        Axis[current_axis].sendCommand(SET_AXIS_REQUESTED_STATE, AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    }
    if (command == "c") {
        if (verb)   Serial.printf("Requesting AXIS_STATE_CLOSED_LOOP_CONTROL in Joint %d\n", current_axis + 1);
        Axis[current_axis].setAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        requestEncoderEstimates();
    }
    if (command == "i") {
        if (verb)   Serial.printf("Requesting AXIS_STATE_IDLE in Joint %d\n", current_axis + 1);
        Axis[current_axis].setAxisRequestedState(AXIS_STATE_IDLE);
    }
    if (command == "p") {
        float pos = Serial.parseFloat();
        pos = constrain(pos, inf_limits_joint[current_axis], sup_limits_joint[current_axis]);
        if (verb)   Serial.printf("input position:\t%.4f\n", pos);
        Axis[current_axis].setInputPos(pos, 0);
    }
    if (command == "a") {
        float accel = Serial.parseFloat();
        if (accel == 0) return;
        if (accel < 0) accel = -accel;
        // float decel = Serial.parseFloat();
        if (verb)   Serial.printf("Traj Acceleration:\t%.4f\n", accel);
        // printf("Traj Deceleration:\t%.4f\n", decel);
        Axis[current_axis].setTrajAccelLimits(accel, accel);
    }
    if (command == "v") {
        float vel = Serial.parseFloat();
        if (vel == 0) return;
        if (vel < 0) vel = -vel;
        if (verb)   Serial.printf("Traj velocity:\t%.4f\n", vel);
        Axis[current_axis].sendFloat(SET_TRAJ_VEL_LIMIT, vel);
    }
    if (command == "g") {
        printf("Requesting for Encoder Estimates ...\n");
        requestEncoderEstimates();
    }
    if (command == "e") {
        int i = current_axis;
        if (verb)   Serial.printf("CLEAR ERRORS for Joint %d\n", i + 1);
        Axis[i].sendCommand(CLEAR_ERRORS, 0);
    }
    if (command == "F") {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (verb)   Serial.printf("Requesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE in Joint %d\n", i + 1);
            Axis[i].sendCommand(SET_AXIS_REQUESTED_STATE, AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
        }
    }
    if (command == "C") {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (verb)   Serial.printf("Requesting AXIS_STATE_CLOSED_LOOP_CONTROL in Joint %d\n", i + 1);
            Axis[i].setAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
        requestEncoderEstimates();
    }
    if (command == "I") {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (verb)   Serial.printf("Requesting AXIS_STATE_IDLE in Joint %d\n", i + 1);
            Axis[i].setAxisRequestedState(AXIS_STATE_IDLE);
        }
    }
    if (command == "E") {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (verb)   Serial.printf("CLEAR ERRORS for Joint %d\n", i + 1);
            Axis[i].sendCommand(CLEAR_ERRORS, 0);
        }
    }
    if (command == "R") { // Enable Reading messages
        read_messages = Serial.parseInt();
        if (read_messages)
            if (verb)   Serial.println("Reading Messages activated");
        else
            if (verb)   Serial.println("Reading Messages disactivated");
    }
    if (command == "S") { // Enable Sending Robot Heartbeat
        send_robot_heartbeat = Serial.parseInt();
        if (send_robot_heartbeat)
            if (verb)   Serial.println("Sending Robot Heartbeat activated");
        else
            if (verb)   Serial.println("Sending Robot Heartbeat disactivated");
    }
    if (command == "?") {
        Serial.printf("Info:\n");
        Serial.printf("\tCurrent_joint:\t%d\n", current_axis + 1);
        Serial.printf("\tCurrent_axis:\t%d\n", current_axis);
        Serial.printf("\tNode_id:\t%d\n", Axis[current_axis].getNodeId());
        Serial.printf("\tTIK\t->\t%.4f", traj_time);
    }
    if (command == "l") {
        if (verb)   Serial.printf("Requesting AXIS_STATE_ENCODER_INDEX_SEARCH in Joint %d\n", current_axis + 1);
        Axis[current_axis].setAxisRequestedState(AXIS_STATE_ENCODER_INDEX_SEARCH);
    }
    if (command == "L") {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (verb)   Serial.printf("Requesting AXIS_STATE_ENCODER_INDEX_SEARCH in Joint %d\n", i + 1);
            Axis[i].setAxisRequestedState(AXIS_STATE_ENCODER_INDEX_SEARCH);
        }
    }
    if (command == "m") {
        if (verb)   Serial.printf("Requesting AXIS_STATE_ENCODER_OFFSET_CALIBRATION in Joint %d\n", current_axis + 1);
        Axis[current_axis].setAxisRequestedState(AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    }
    if (command == "M") {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (verb)   Serial.printf("Requesting AXIS_STATE_ENCODER_OFFSET_CALIBRATION in Joint %d\n", i + 1);
            Axis[i].setAxisRequestedState(AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
        }
    }
    if (command == "h") {
        Axis[current_axis].setTrajAccelLimits(10, 10);
        Axis[current_axis].sendFloat(SET_TRAJ_VEL_LIMIT, 10);
        if (verb)   Serial.printf("Requesting AXIS_STATE_HOMING in Joint %d\n", current_axis + 1);
        Axis[current_axis].setAxisRequestedState(AXIS_STATE_HOMING);
    }
}

void readMessagesFromODrive(void) {
    CAN_message_t msg;
    if ( can1.read(msg) ) {
        byte nodeId = msg.id >> 5;
        byte cmdId = msg.id & 0b11111;
        // Serial.printf("nodeID: %d,\tcmdID: %d\n", nodeId, cmdId);
        if (cmdId == ODRIVE_HEARTBEAT_MESSAGE) {
            // Serial.println("heartbeat");
            if (nodeId == (current_axis + 1)) {
                // Serial.println("Leyendo");
                for (int i = 0; i < 8; i++) {
                    if (i < 4) axis_error.iBuff[i] = msg.buf[i];
                    else axis_current_state.iBuff[i-4] = msg.buf[i];
                }

                if (Axis[current_axis].getAxisCurrentState() != Axis[current_axis].getAxisPrevState()) {
                    // static int cont = 0;
                    if (verb)   Serial.printf("Change,\t\tinit_secuence_flag: %d\n", _init_secuence_flag);
                    cont++;
                    if (cont == 2)  {
                        _axis_state_changed = true;
                        cont = 0;
                    }
                }

                Axis[nodeId-1].setAxisError(axis_error.i);
                Axis[nodeId-1].setAxisPrevState(Axis[nodeId-1].getAxisCurrentState());
                Axis[nodeId-1].setAxisCurrentState(msg.buf[4]);

                // if (current_axis/* nodeId */ == 1)    if (verb)   showHeartbeatCANMessage(msg);
                if (verb && read_messages) showHeartbeatCANMessage(msg);

                if (_init_secuence_flag && _axis_state_changed) {
                    init_secuence_step++;
                    _axis_state_changed = false;
                    if (init_secuence_step == 1) {
                        if (verb)   Serial.printf("AXIS_STATE_ENCODER_INDEX_SEARCH\tstep:  %d\n", init_secuence_step);
                        if (current_axis == 1) {
                            _axis_state_changed = true;
                        }
                        Axis[current_axis].setAxisRequestedState(AXIS_STATE_ENCODER_INDEX_SEARCH);
                    }
                    if (init_secuence_step == 2) {
                        if (verb)   Serial.printf("AXIS_STATE_ENCODER_OFFSET_CALIBRATION\tstep:  %d\n", init_secuence_step);
                        Axis[current_axis].sendCommand(CLEAR_ERRORS, 0);    delay(7);
                        Axis[current_axis].setAxisRequestedState(AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
                    }
                    if (init_secuence_step == 3) {
                        Axis[current_axis].setTrajAccelLimits(15, 15);
                        Axis[current_axis].sendFloat(SET_TRAJ_VEL_LIMIT, 50);
                        Axis[current_axis].sendCommand(CLEAR_ERRORS, 0);    delay(7);
                        if (verb)   Serial.printf("AXIS_STATE_HOMING\tstep:  %d\n", init_secuence_step);
                        Axis[current_axis].setAxisRequestedState(AXIS_STATE_HOMING);
                    }
                    if (init_secuence_step == 4) {
                        Axis[current_axis].setTrajAccelLimits(250, 250);
                        Axis[current_axis].sendFloat(SET_TRAJ_VEL_LIMIT, 50);
                        Axis[current_axis].sendCommand(CLEAR_ERRORS, 0);    delay(7);
                        if (verb)   Serial.printf("AXIS_STATE_CLOSED_LOOP_CONTROL\tstep:  %d\n", init_secuence_step);
                        Axis[current_axis].setAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);
                        _init_secuence_flag = false;
                        init_secuence_step = 0;
                        requestEncoderEstimates();
                        if (verb)   Serial.println("Finished");
                    }
                }
            }
        }
        
        if (cmdId == GET_ENCODER_ESTIMATES) {
            byte k = 0;
            dataUnion position, velocity;
            for (int i = 0; i < 8; i++) {
                if (i < 4) {
                    position.fBuff[i] = msg.buf[i];
                } else {
                    velocity.fBuff[i-4] = msg.buf[i];
                }
            }
            static int verif = 0;
            for (int i = 1; i <= NUM_JOINTS; i++) {
                if (nodeId == i) {
                    Qencoder[i-1] = position.f*Axis[i-1].getSpinDirection()*(2*M_PI)/reduction_ratios[i-1];
                    dQencoder[i-1] = velocity.f*Axis[i-1].getSpinDirection()*(2*M_PI)/reduction_ratios[i-1];
                    verif++;
                }
            }
            if (verif == 6) {
                verif = 0;
                // Serial.println(verif);
                encoderStimatesFinished = true;
                // Qc = arr2art(Qencoder);
                if (catch_point_flag) {
                    Serial.println("Has cachado un punto");
                    art Q = dotART(RAD_TO_DEG, arr2art(Qencoder));
                    for (int i = 0; i < NUM_POINTS; i++) {
                        if (points[i].tag == point_name) {
                            points[i].Q = Q;
                            i = NUM_POINTS;
                        }
                        if (points[i].tag == "") {
                            points[i].tag = point_name;
                            points[i].Q = Q;
                            i = NUM_POINTS;
                        }
                    }
                    point_name = "";
                    catch_point_flag = false;
                }
                // Serial.printf(",%f\n",
                //             Qencoder[5]*RAD_TO_DEG/* Axis[4].getSpinDirection()*360/REDUCTION_RATIO_JOINT5 */);
                // Serial.printf(",%f, %f, %f, %f, %f, %f\n",
                //             Qencoder[0]*RAD_TO_DEG/* Axis[0].getSpinDirection()*360/REDUCTION_RATIO_JOINT1 */,
                //             Qencoder[1]*RAD_TO_DEG/* Axis[1].getSpinDirection()*360/REDUCTION_RATIO_JOINT2 */,
                //             Qencoder[2]*RAD_TO_DEG/* Axis[2].getSpinDirection()*360/REDUCTION_RATIO_JOINT3 */,
                //             Qencoder[3]*RAD_TO_DEG/* Axis[3].getSpinDirection()*360/REDUCTION_RATIO_JOINT4 */,
                //             Qencoder[4]*RAD_TO_DEG/* Axis[4].getSpinDirection()*360/REDUCTION_RATIO_JOINT5 */,
                //             Qencoder[5]*RAD_TO_DEG/* Axis[5].getSpinDirection()*360/REDUCTION_RATIO_JOINT6 */);
                // Serial.printf("_Q%f, %f, %f, %f, %f, %f\n",
                //             Qencoder[0]*RAD_TO_DEG/* Axis[0].getSpinDirection()*360/REDUCTION_RATIO_JOINT1 */,
                //             Qencoder[1]*RAD_TO_DEG/* Axis[1].getSpinDirection()*360/REDUCTION_RATIO_JOINT2 */,
                //             Qencoder[2]*RAD_TO_DEG/* Axis[2].getSpinDirection()*360/REDUCTION_RATIO_JOINT3 */,
                //             Qencoder[3]*RAD_TO_DEG/* Axis[3].getSpinDirection()*360/REDUCTION_RATIO_JOINT4 */,
                //             Qencoder[4]*RAD_TO_DEG/* Axis[4].getSpinDirection()*360/REDUCTION_RATIO_JOINT5 */,
                //             Qencoder[5]*RAD_TO_DEG/* Axis[5].getSpinDirection()*360/REDUCTION_RATIO_JOINT6 */);
                pose X1 = DK(arr2art(Qencoder), 0, 0, 0);
                pose X2 = DK(arr2art(Qencoder), 0, 0, 1);
                mth T = dirKinematics(Qc);
                Tc = T;
                // Serial.printf(", %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", X1.p.x, X1.p.y, X1.p.z,
                //             X1.o.x*RAD_TO_DEG, X1.o.y*RAD_TO_DEG, X1.o.z*RAD_TO_DEG);
                // Serial.printf("_X%f, %f, %f, %f, %f, %f, %f, %f, %f\n", X1.p.x, X1.p.y, X1.p.z,
                //             X1.o.x*RAD_TO_DEG, X1.o.y*RAD_TO_DEG, X1.o.z*RAD_TO_DEG,
                //             X2.o.x*RAD_TO_DEG, X2.o.y*RAD_TO_DEG, X2.o.z*RAD_TO_DEG);
                // Serial.printf("_T%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                //             T.p.x, T.p.y, T.p.z, T.R.r11, T.R.r12, T.R.r13,
                //             T.R.r21, T.R.r22, T.R.r23, T.R.r31, T.R.r32, T.R.r33);
            }

            // Serial.printf("%f, ", position.f);
            // if (nodeId == 6)    Serial.println("fin");

            // if (nodeId == 1)    Qc.q1 = position.f;
            // if (nodeId == 2)    Qc.q2 = position.f;
            // if (nodeId == 3)    Qc.q3 = position.f;
            // if (nodeId == 4)    Qc.q4 = position.f;
            // if (nodeId == 5)    Qc.q5 = position.f;
            // if (nodeId == 6)    Qc.q6 = position.f;
        }
    }
}

void showHeartbeatCANMessage(CAN_message_t msg) {
    static unsigned long _m = 0, _prevM = 0;
    Serial.printf("ODrive Heartbeat\tJoint: %d\tMessage: 0x", msg.id >> 5);
    for (int i = 0; i < 4; i++) {
        int num = msg.buf[7 - i - 4];
        Serial.print(" ");
        if (num < 16)   Serial.print("0");
        Serial.print(num, HEX);
    }
    Serial.print(" ");
    for (int i = 0; i < 4; i++) {
        int num = msg.buf[7 - i];
        Serial.print(" ");
        if (num < 16)   Serial.print("0");
        Serial.print(num, HEX);
    }
    Serial.printf("\tTS: "); Serial.print((_m = millis()) - _prevM);
    Serial.printf("\t\tstep: ");  Serial.print(init_secuence_step);
    Serial.printf("\t\tstate: ");  Serial.print(Axis[current_axis].getAxisCurrentState());
    Serial.printf("\tprev_state: "); Serial.print(Axis[current_axis].getAxisPrevState());
    Serial.printf("\tcont: "); Serial.print(cont);
    Serial.println();
    _prevM = _m;
}





/* Functions */
void counterUp() {
  // Serial.println(counts++);
  // if (varFlag >= 1 && varFlag <=3) {
  //   if (varFlag == 1) Xc.p.x += 1;
  //   if (varFlag == 2) Xc.p.y += 1;
  //   if (varFlag == 3) Xc.p.z += 1;
  //   if (verb) printf("X: %.4f, Y: %.4f, Z: %.4f, A: %.4f, B: %.4f, C: %.4f\n", Xc.p.x, Xc.p.y, Xc.p.z, Xc.o.x, Xc.o.y, Xc.o.z);
  //   Qc = ikSolvePTP(Xc, Qc);
  //   commandJoints(Qc);
  // }
  // if (varFlag == 4) {
  //   Serial2.print("p");
  //   switch (current_joint) {
  //     case 1: { Qc.q1 += M_PI/180.0; Serial2.println(Qc.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1); } break;
  //     case 2: { Qc.q2 += M_PI/180.0; Serial2.println(Qc.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2); } break;
  //     case 3: { Qc.q3 += M_PI/180.0; Serial2.println(Qc.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3); } break;
  //     case 4: { Qc.q4 += M_PI/180.0; Serial2.println(Qc.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4); } break;
  //     case 5: { Qc.q5 += M_PI/180.0; Serial2.println(Qc.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5); } break;
  //     case 6: { Qc.q6 += M_PI/180.0; Serial2.println(Qc.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6); } break;
  //     default:  break;
  //   }
  // }
  // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void counterDown() {
  // Serial.println(counts--);
  // if (varFlag >= 1 && varFlag <=3) {
  //   if (varFlag == 1) Xc.p.x -= 1;
  //   if (varFlag == 2) Xc.p.y -= 1;
  //   if (varFlag == 3) Xc.p.z -= 1;
  //   if (verb) printf("X: %.4f, Y: %.4f, Z: %.4f, A: %.4f, B: %.4f, C: %.4f\n", Xc.p.x, Xc.p.y, Xc.p.z, Xc.o.x, Xc.o.y, Xc.o.z);
  //   Qc = ikSolvePTP(Xc, Qc);
  //   commandJoints(Qc);
  // }
  // if (varFlag == 4) {
  //   Serial2.print("p");
  //   switch (current_joint) {
  //     case 1: { Qc.q1 -= M_PI/180.0; Serial2.println(Qc.q1/(2*M_PI)*REDUCTION_RATIO_JOINT1); } break;
  //     case 2: { Qc.q2 -= M_PI/180.0; Serial2.println(Qc.q2/(2*M_PI)*REDUCTION_RATIO_JOINT2); } break;
  //     case 3: { Qc.q3 -= M_PI/180.0; Serial2.println(Qc.q3/(2*M_PI)*REDUCTION_RATIO_JOINT3); } break;
  //     case 4: { Qc.q4 -= M_PI/180.0; Serial2.println(Qc.q4/(2*M_PI)*REDUCTION_RATIO_JOINT4); } break;
  //     case 5: { Qc.q5 -= M_PI/180.0; Serial2.println(Qc.q5/(2*M_PI)*REDUCTION_RATIO_JOINT5); } break;
  //     case 6: { Qc.q6 -= M_PI/180.0; Serial2.println(Qc.q6/(2*M_PI)*REDUCTION_RATIO_JOINT6); } break;
  //     default:  break;
  //   }
  // }
  // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}
