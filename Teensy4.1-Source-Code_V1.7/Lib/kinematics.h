#ifndef KINEMATICS_H_
#define KINEMATICS_H_

// #include <stdio.h>
// #include <LibPrintf.h>
#include "spatial.c"

#define zeros	(vec) { 0, 0, 0 }
#define ones	(vec) { 1, 1, 1 }
#define zeros3	(mat) { 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define ones3	(mat) { 1, 1, 1, 1, 1, 1, 1, 1, 1 }
#define eye		(mat) { 1, 0, 0, 0, 1, 0, 0, 0, 1 }
#define zeros6 	{ 0, 0, 0, 0, 0, 0 };
#define abs(n)	n < 0 ? -n : n

// Kuka KR3 Geometry
#define L1	345
#define L2	 20
#define L3	260
#define L4	 20
#define L5	260
#define L6	100//75

// #define L1	330
// #define L2	 50
// #define L3	330
// #define L4	 35
// #define L5	335
// #define L6	160

// DH-parameters
double   th[] = {	  0, pi/2,    0, 	  0, 	0,  0};
double 	  d[] = {	 L1,    0,    0, 	 L5, 	0, L6};
double 	  a[] = { 	 L2,   L3,   L4, 	  0, 	0,  0};
double alph[] = { pi/2,    0, pi/2,  -pi/2, pi/2,  0};
// Link mass
double m[] = { 8558.13,
			   3222.21,
			   2441.35,
			   3720.80,
			    897.21,
				529.25};
vec c[] = { { -10.24,   1.03, 107.64 },
		 	{ -17.74, -67.23,  -9.04 },
		 	{-127.32,  14.13, -16.01 },
		 	{ -12.18,  -0.03,  28.46 },
		 	{	0.76,  82.20,  -8.24 },
		 	{	0.11, 	1.71,  32.74 }};
mat I[] = { { 192218691.57, 	-16732.88,  -8587986.22,
			  	 -16732.88,  206629402.59, 	  787266.65,
			   -8587986.22, 	787266.65, 106474370.26 },
			{  35023530.88,    5769504.38,    -75650.98,
				5769504.38,   14318458.49, 	  -43745.05,
				 -75650.98, 	-43745.05,  33693976.72 },
			{	9373424.88,   -4247373.09, 	 2248918.33,
			   -4247373.09,   67653342.32, 	 -491856.55,
				2248918.33,    -491856.55,  64118877.02 },
			{  13542349.09, 	  -413.44, 	  -26625.91,
			  	   -413.44,   14652021.88, 	   -4023.59,
			  	 -26625.91, 	 -4023.59,   6662824.99 },
			{  10054567.11, 	 63260.57, 	  -19035.94,
			  	  63260.57,    1644497.25, 	 -213786.90,
			  	 -19035.94,    -213786.90,   9573380.70 },
			{ 	1363864.36, 	  1263.16, 		1465.31,
			  	   1263.16,    1366972.95, 		1359.32,
			  	   1465.31, 	  1359.32, 	  396346.58 }};

const vec r[] = {{a[0], d[0]*sin(alph[0]), d[0]*cos(alph[0])},
				 {a[1], d[1]*sin(alph[1]), d[1]*cos(alph[1])},
				 {a[2], d[2]*sin(alph[2]), d[2]*cos(alph[2])},
				 {a[3], d[3]*sin(alph[3]), d[3]*cos(alph[3])},
				 {a[4], d[4]*sin(alph[4]), d[4]*cos(alph[4])},
				 {a[5], d[5]*sin(alph[5]), d[5]*cos(alph[5])}};

typedef struct {
	mat R;
	vec p;
} mth;

typedef struct {
	double q1;
	double q2;
	double q3;
	double q4;
	double q5;
	double q6;
} art;

typedef struct {
	vec p;
	vec o;
} pose;

typedef struct {
	art Jvx;
	art Jvy;
	art Jvz;
	art Jwx;
	art Jwy;
	art Jwz;
} jac;

mth TCP = {eye, {0, 0, 0}};

art addART(art,art);
art subART(art,art);
art dotART(double, art);
pose addPOSE(pose,pose);
pose subPOSE(pose,pose);
double normART(art);
double normPOSE(pose);

mth dotMTH(mth,mth);
mth dh(double,double,double,double);
mth dirKinematics(art);
art invKinematics(mth,bool,bool,bool);
pose DK(art,short,short,bool);
mth pose2mth(pose, short, short);
art IK(pose, short, short, bool, bool, bool);
mth invMTH(mth mth);

void printMTH(mth);
void printlnMTH(mth);
void printART(art);
void printlnART(art);
void printPOSE(pose);
void printlnPOSE(pose);


#endif /* KINEMATICS_H_ */
