#include "planning.h"
#include <math.h>

/* rad2deg:	Function that converts radians to degrees units of a 3D vector */
vec rad2deg(vec v)
{
	double factor = 180.0 / pi;
	v.x *= factor;
	v.y *= factor;
	v.z *= factor;
	return v;
}

/* deg2rad:	Function that converts degrees to radians units of a 3D vector */
vec deg2rad(vec v)
{
	double factor = pi / 180.0;
	v.x *= factor;
	v.y *= factor;
	v.z *= factor;
	return v;
}

/* cubic - Cubic Interpolator
	This ROBOT function creates a 6-by-N-by-4 Matrix that contains a cubic
	interpolation Data.
	cubic(T) */
parameter _cubic(double T, double t)
{
	static double T2, T3;
	static double pos, vel, acell, jerk;
	if (t == 0.0) {
		T2 = T*T;
		T3 = T2*T;
	}
	// Interpolation
	double t2 = t*t;
	double t3 = t2*t;
	pos 	= 3.0 * t2 / T2 - 2.0 * t3 / T3;
	vel 	= 6.0 * t / T2 - 6.0 * t2 / T3;
	acell 	= 6.0 / T2 - 12.0 * t / T3;
	jerk	=             - 12.0/T3;
	return (parameter) {pos, vel, acell, jerk};
}

/* quintic - Quintic Interpolator
	This ROBOT function creates a 6-by-N-by-4 Matrix that contains a
	quintic interpolation Data.
	quintic(T) */
parameter _quintic(double T, double t)
{
	static double T3, T4, T5;
	static double pos, vel, acell, jerk;
	if (t == 0.0) {
		T3 = T*T*T;
		T4 = T3*T;
		T5 = T4*T;
	}
	// Interpolation
	double t2 = t*t;
	double t3 = t2*t;
	double t4 = t3*t;
	double t5 = t4*t;
	pos		= 10.0*t3/T3 -  15.0*t4/T4 +   6.0*t5/T5;
	vel		= 30.0 * t2 / T3 - 60.0 * t3 / T4 + 30.0 * t4 / T5;
	acell	= 60.0 * t / T3 - 180.0 * t2 / T4 + 120.0 * t3 / T5;
	jerk	= 60.0 / T3 - 360.0 * t / T4 + 360.0 * t2 / T5;
	return (parameter) {pos, vel, acell, jerk};
}

/* trapezoidal - Trapezoidal velocity profile Interpolator
	This ROBOT function creates a 6-by-N-by-4 Matrix that contains a
	trapezoidal velocity profile interpolation Data.
	trapezoidal(pv,T) */
parameter _trapezoidal(double pv, double T, double t)
{
	static double tc, tb, vc, ac;
	static double pos, vel, acell, jerk;
	if (t == 0.0) {
		tc = T - T/(2.0*pv);
		tb = T - tc;
		vc = 2.0*pv/T;
		ac = vc/tc;
	}
	// Interpolation
	if (t <= tc) {
		pos   = 0.5*ac*t*t;
		vel   = ac * t;
		acell = ac;
		jerk  = 0;
	} else if (t <= tb) {
		pos   = vc * (t - 0.5 * tc);
		vel   = vc;
		acell = 0;
		jerk  = 0;
	} else {
		double T_t = T - t;
		pos   = 1.0 - 0.5 * ac * T_t * T_t;
		vel   = ac * T_t;
		acell = -ac;
		jerk  = 0;
	}
	return (parameter) {pos, vel, acell, jerk};
}

/* sCurve - Trapezoidal velocity profile Interpolator (sin)
	This ROBOT function creates a 6-by-N-by-4 Matrix that contains a
	trapezoidal velocity profile with S-curve(sin function in jerk)
	Interpolation Data.
	sCurve(pv,T) */
parameter _sCurve(double pv, double T, double t)
{
	static double tc, tb, vc, ac, pi2, tc2;
	static double pos, vel, acell, jerk;
	if (t == 0.0) {
		tc = T - T/(2.0*pv);
		tb = T - tc;
		vc = 2.0*pv/T;
		ac = vc/tc;
		pi2 = pi*pi;
		tc2 = tc*tc;
	}
	// Interpolation
	if (t <= tc) {
		pos		= vc*(t*t/(2.0*tc) + tc/(4.0*pi2)*cos(2.0*pi/tc*t) - tc/(4.0*pi2));
		vel		= vc*(t/tc - 0.5/pi*sin(2.0*pi/tc*t));
		acell	= vc/tc*(1.0 - cos(2.0*pi/tc*t));
		jerk	= 2.0*pi*vc/tc2*sin(2.0*pi/tc*t);
	} else if (t <= tb) {
		pos		= vc*(t-0.5*tc);
		vel		= vc;
		acell	= 0;
		jerk	= 0;
	} else {
		double T_t = T - t;
		pos		=  1.0 - vc*(T_t*T_t/(2.0*tc) + tc/(4.0*pi2)*cos(2.0*pi/tc*T_t) - tc/(4.0*pi2));
		vel		=  vc*(T_t/tc - 0.5/pi*sin(2.0*pi/tc*T_t));
		acell	= -vc/tc*(1.0 - cos(2.0*pi/tc*T_t));
		jerk	=  2.0*pi*vc/tc2*sin(2.0*pi/tc*T_t);
	}
	return (parameter) {pos, vel, acell, jerk};
}

/* s2Curve - Trapezoidal velocity profile Interpolator (sin^2)
	This ROBOT function creates a 6-by-N-by-4 Matrix that contains a
	trapezoidal velocity profile with S-curve(sin^2 function in jerk)
	Interpolation Data.
	s2Curve(pv,T) */
parameter _s2Curve(double pv, double T, double t)
{
	static double tc, tcc, tbb, tb, vc, ac, pi2, pi3, tc2, tc3;
	static double pos, vel, acell, jerk;
	if (t == 0.0) {
		tc = T - T/(2.0*pv);
		tb = T - tc;
		tcc = tc/2;
		tbb = T - tcc;
		vc = 2.0*pv/T;
		ac = vc/tc;
		pi2 = pi*pi;
		pi3 = pi2*pi;
		tc2 = tc*tc;
		tc3 = tc2*tc;
	}
	// Interpolation
	if (t <= tcc) {
			pos		= 2*vc/tc2*(t*t*t/3 + tc3/(32*pi3)*sin(4*pi/tc*t) - tc2/(8*pi2)*t);
			vel		= 2*vc/tc2*(t*t + tc2/(8*pi2)*(cos(4*pi/tc*t) - 1));
			acell	= 4*vc/tc2*(t - tc/(4*pi)*sin(4*pi/tc*t));
			jerk	= 8*vc/tc2*sin(2*pi/tc*t)*sin(2*pi/tc*t);
		} else if (t <= tc) {
			double tc_t = tc - t;
			pos		=  vc*(t-tcc) + 2*vc/tc2*(tc_t*tc_t*tc_t/3 + tc3/(32*pi3)*sin(4*pi/tc*tc_t) - tc2/(8*pi2)*tc_t);
			vel		=  vc - 2*vc/tc2*(tc_t*tc_t + tc2/(8*pi2)*(cos(4*pi/tc*tc_t) - 1));
			acell	=  4*vc/tc2*(tc_t - tc/(4*pi)*sin(4*pi/tc*tc_t));
			jerk	= -8*vc/tc2*sin(2*pi/tc*tc_t)*sin(2*pi/tc*tc_t);
		} else if (t <= tb) {
			pos		= vc*(t-0.5*tc);
			vel		= vc;
			acell	= 0;
			jerk	= 0;
		} else if (t <= tbb) {
			double t_tb = t - tb;
			pos		=  1.0 - vc*(tbb-t) - 2*vc/tc2*(t_tb*t_tb*t_tb/3 + tc3/(32*pi3)*sin(4*pi/tc*t_tb) - tc2/(8*pi2)*t_tb);
			vel		=  vc - 2*vc/tc2*(t_tb*t_tb + tc2/(8*pi2)*(cos(4*pi/tc*t_tb) - 1));
			acell	= -4*vc/tc2*(t_tb - tc/(4*pi)*sin(4*pi/tc*t_tb));
			jerk	= -8*vc/tc2*sin(2*pi/tc*t_tb)*sin(2*pi/tc*t_tb);
		} else {
			double T_t = T - t;
			pos		=  1.0 - 2*vc/tc2*(T_t*T_t*T_t/3 + tc3/(32*pi3)*sin(4*pi/tc*T_t) - tc2/(8*pi2)*T_t);
			vel		=  2*vc/tc2*(T_t*T_t + tc2/(8*pi2)*(cos(4*pi/tc*T_t) - 1));
			acell	= -4*vc/tc2*(T_t  - tc/(4*pi)*sin(4*pi/tc*T_t ));
			jerk	=  8*vc/tc2*sin(2*pi/tc*T_t)*sin(2*pi/tc*T_t);
		}
	return (parameter) {pos, vel, acell, jerk};
}

parameter _hsCurve(double T, double t)
{
	double pa = 0.5, pv = 0.5;
	static double Vmin, V, Amin, A, J, w;
	static double t1, t2, t3, t4, t5, t6, t7;
	static double pos, vel, acell, jerk;
	if (t == 0.0) {
		t7 = T;
		t3 = t7*(1-1/(pv+1));
		t1 = t3*(1-1/(pa+1));
		t2 = t3 - t1;
		t4 = t7 - t3;
		t5 = t4 + t1;
		t6 = t7 - t1;
		Vmin = 1/t7;
		V = Vmin*(1+pv);
		Amin = V/t3;
		A = Amin*(1+pa);
		J = 2*A/t1;
		w = 2*pi/t1;
	}
	// Interpolation
	if (t <= t1) {
		pos		= J/2*(pow(t,3)/6 - t/pow(w,2) + sin(w*t)/pow(w,3));
		vel		= J/2*(pow(t,2)/2 + (cos(w*t) - 1)/pow(w,2));
		acell	= J/2*(t - sin(w*t)/w);
		jerk	= J/2*(1 - cos(w*t));
	} else if (t <= t2) {
		pos		= A/2*(pow(t,2) - t1*t + pow(t1,2)/3 - 2/pow(w,2));
		vel		= A*(t - t1/2);
		acell	= A;
		jerk	= 0;
	} else if (t <= t3) {
		pos		= A*(pow(t,2)/2 - (t1*t)/2 + pow(t1,2)/6 - 1/pow(w,2)) - J/2*((pow(t-t2,3))/6 + sin(w*(t - t2))/pow(w,3) - (t - t2)/pow(w,2));
		vel		= A*(t - t1/2) - J/2*(pow(t-t2,2)/2 + (cos(w*(t-t2)) - 1)/pow(w,2));
		acell	= A - J/2*(t-t2-sin(w*(t-t2))/w);
		jerk	= -J/2*(1-cos(w*(t-t2)));
	} else if (t <= t4) {
		pos		= V*(t-t3/2);
		vel		= V;
		acell	= 0;
		jerk	= 0;
	} else if (t <= t5) {
		pos		= V*(t-t3/2) - J/2*(pow(t-t4,3)/6 + sin(w*(t-t4))/pow(w,3) - (t-t4)/pow(w,2));
		vel		= V - J/2*(pow(t-t4,2)/2 + (cos(w*(t-t4)) - 1)/pow(w,2));
		acell	= -J/2*(t - t4 - sin(w*(t-t4))/w);
		jerk	= -J/2*(1-cos(w*(t-t4)));
	} else if (t <= t6) {
		pos		= V*(t-t3/2) - A*(pow(t-t5,2)/2 + t1*(t-t5)/2 + pow(t1,2)/6 - 1/pow(w,2));
		vel		= V - A*(t + t1/2 - t5);
		acell	= -A;
		jerk	= 0;
	} else {
		pos		= 1 + V*(t - t7) - A*((pow(t,2) - pow(t7,2))/2 + (t1/2 - t5)*(t - t7)) + J/2*(pow(t-t6,3)/6 + sin(w*(t - t6))/pow(w,3) - (t-t7)/pow(w,2) - pow(t1,3)/6);
		vel		= V - A*(t + t1/2 - t5) + J/2*(pow(t-t6,2)/2 + (cos(w*(t-t6)) - 1)/pow(w,2));
		acell	= J/2*(t-t6 - sin(w*(t-t6))/w) - A;
		jerk	= J/2*(1-cos(w*(t-t6)));
	}
	return (parameter) {pos, vel, acell, jerk};
}
