#ifndef KINEMATIC_CONTROL_H_
#define KINEMATIC_CONTROL_H_

// #include <stdio.h>
// #include <LibPrintf.h>
#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include "planning.c"

art Q0, Qf, dQ;
pose X0, Xf, Xa;
vec p0, dP, k;
mat R0;
double theta;
vec C_, ri, n;
double ang;
bool is_move_position = true;
bool is_move_orientation = true;

art Qc = {0, 0, 0, 0, 0, 0};
art Qc_ant = {0, 0, 0, 0, 0, 0};
art dQc = {0, 0, 0, 0, 0, 0};
art dQc_ant = {0, 0, 0, 0, 0, 0};
art ddQc = {0, 0, 0, 0, 0, 0};
art dddQc = {0, 0, 0, 0, 0, 0};
art tauc = {0, 0, 0, 0, 0, 0};
art Qhome = {0, 0, 0, 0, 0, 0};
pose Xc = {{L2 + L5 + L6, 0, L1 + L3 + L4}, {0, -pi / 2, pi}};
pose Xhome = {{L2 + L5 + L6, 0, L1 + L3 + L4}, {0, -pi / 2, pi}};
pose Phome = {{L2 + L5 + L6, 0, L1 + L3 + L4}, {0, -90, 180}};

bool printflag = false;

art maxArt = {
	1.00 * pi,
	0.41 * pi,
	1.05 * pi,
	1.05 * pi,
	0.75 * pi,
	2.00 * pi,
};

art minArt = {
	-1.00 * pi,
	-0.72 * pi,
	-0.31 * pi,
	-1.05 * pi,
	-0.75 * pi,
	-2.00 * pi,
};

/* Prototypes */
art ikSolvePTP(pose, art);
art ikSolve(pose X, art Qant);
art ikSolveMTH(mth X, art Qant);
//void PTP(pose,double,double,char);
void _PTP(pose P, ...);
void _MOVE(art Q, ...);
void _PTPMotionHandler();
void _LIN(pose P, ...);
void _LINMotionHandler();
void _HOME(double T);
bool someNanInJoint(art _Q);
art isnanFix(art _Q);

art Dynamics(art q, art _qp, art _qpp, vec Fe, vec Me);

#endif /* KINEMATIC_CONTROL_H_ */
