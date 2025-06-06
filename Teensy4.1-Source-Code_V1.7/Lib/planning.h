#ifndef PLANNING_H_
#define PLANNING_H_

#include <stdlib.h>
#include "kinematics.c"

//#define bool 	int

#define Tm	0.01

typedef struct {
	double pos;
	double vel;
	double acell;
	double jerk;
} parameter;

unsigned int index_0;
unsigned int index_f;
int samples, i;
double pv, pa, T;
char ip;
#define TRAJ_NONE_FLAG		0x0
#define TRAJ_PTP_FLAG		0x1
#define TRAJ_LIN_FLAG		0x2
#define TRAJ_CIRC_FLAG		0x3
#define TRAJ_FINISH_FLAG	0x10
char trajFlags = TRAJ_NONE_FLAG;

double _time = -Tm;

vec rad2deg(vec);
vec deg2rad(vec);
parameter _cubic(double T, double t);
parameter _quintic(double T, double t);
parameter _trapezoidal(double pv, double T, double t);
parameter _sCurve(double pv, double T, double t);
parameter _s2Curve(double pv, double T, double t);
parameter _hsCurve(double T, double t);


#endif /* PLANNING_H_ */
