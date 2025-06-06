#include "Control.h"

art ikSolvePTP(pose X, art Qant)
{
	art Q[9];
	double ant, act;
	int i, iRes = -1, ind = 0;
	Q[7] = IK(X, 0, 0, 1, 1, 1);
	Q[6] = IK(X, 0, 0, 1, 1, 0);
	Q[5] = IK(X, 0, 0, 1, 0, 1);
	Q[4] = IK(X, 0, 0, 1, 0, 0);
	Q[3] = IK(X, 0, 0, 0, 1, 1);
	Q[2] = IK(X, 0, 0, 0, 1, 0);
	Q[1] = IK(X, 0, 0, 0, 0, 1);
	Q[0] = IK(X, 0, 0, 0, 0, 0);
	for (i = 0; i < 8; i++)
	{
		int cond = true;
		art Qi = Q[i];
		// cond = cond && Qi.q1 > minArt.q1 && Qi.q1 < maxArt.q1;
		// cond = cond && Qi.q2 > minArt.q2 && Qi.q2 < maxArt.q2;
		// cond = cond && Qi.q3 > minArt.q3 && Qi.q3 < maxArt.q3;
		// cond = cond && Qi.q4 > minArt.q4 && Qi.q4 < maxArt.q4;
		// cond = cond && Qi.q5 > minArt.q5 && Qi.q5 < maxArt.q5;
		// cond = cond && Qi.q6 > minArt.q6 && Qi.q6 < maxArt.q6;
		/* If ik solution is valid */
		if (cond) 
		{
			act = normART(subART(Qi, Qant));
			if (ind > 0)
				if (act < ant)
					iRes = i;
			ant = act;
			ind++;
		}
	}
	return Q[iRes];
}

art ikSolve(pose X, art Qant) {
	art Q[8];
	double min;
	int i, ind = 0;
	Q[0] = IK(X, 0, 0, 1, 1, 1);
	Q[1] = IK(X, 0, 0, 1, 1, 0);
	Q[2] = IK(X, 0, 0, 1, 0, 1);
	Q[3] = IK(X, 0, 0, 1, 0, 0);
	Q[4] = IK(X, 0, 0, 0, 1, 1);
	Q[5] = IK(X, 0, 0, 0, 1, 0);
	Q[6] = IK(X, 0, 0, 0, 0, 1);
	Q[7] = IK(X, 0, 0, 0, 0, 0);
	min = normART(subART(Q[0], Qant));
	for (i = 0; i < 8; i++) {
		double d = normART(subART(Q[i], Qant));
		if (d < min) {
			min = d;
			ind = i;
		}
	}
	return Q[ind];
}

art ikSolveMTH(mth T, art Qant) {
	art Q[8];
	double min;
	int i, ind = 0;
	Q[0] = invKinematics(T, 1, 1, 1);
	Q[1] = invKinematics(T, 1, 1, 0);
	Q[2] = invKinematics(T, 1, 0, 1);
	Q[3] = invKinematics(T, 1, 0, 0);
	Q[4] = invKinematics(T, 0, 1, 1);
	Q[5] = invKinematics(T, 0, 1, 0);
	Q[6] = invKinematics(T, 0, 0, 1);
	Q[7] = invKinematics(T, 0, 0, 0);
	min = normART(subART(Q[0], Qant));
	for (i = 0; i < 8; i++) {
		double d = normART(subART(Q[i], Qant));
		if (d < min) {
			min = d;
			ind = i;
		}
	}
	return Q[ind];
}

/* PTP - Point to Point Motion
	This ROBOT command creates a 6-by-N-by-4 Matrix that contains a
	point-to-point trajectory to a final robot pose. Sinusoidal
	jerk S curve interpolation as default.
	PTP(P,t,pv) */
void _PTP(pose P, ...)
{
	T = 1.0; pv = 0.8; ip = 'h';
	va_list ap;
	va_start(ap, P);
	T = va_arg(ap, double);
	pv = va_arg(ap, double);
	ip = va_arg(ap, int);
	va_end(ap);
	Xf = (pose){P.p, deg2rad(P.o)};
	Q0 = Qc;
	Qf = ikSolvePTP(Xf, Qc);
	dQ = subART(Qf, Q0);

	i = 0;
	samples = (int) (T/Tm + 1);

	trajFlags = TRAJ_PTP_FLAG;
}

void _MOVE(art Qf, ...)
{
	T = 1.0; pv = 0.8; pa = 0.8; ip = 'h';
	va_list ap;
	va_start(ap, Qf);
	T = va_arg(ap, double);
	pv = va_arg(ap, double);
	ip = va_arg(ap, int);
	va_end(ap);
	Q0 = Qc;
	dQ = subART(Qf, Q0);

	i = 0;
	samples = (int) (T/Tm + 1);

	trajFlags = TRAJ_PTP_FLAG;
}

void _PTPMotionHandler()
{
	static double /* t = 0.0,  */_t = 0.0;
	// unsigned long long t0 = micros();
	if (!i) {
		_t = 0.0;
		Q0 = Qc;
		X0 = Xf;
	}
	if (i == samples) {
		trajFlags = TRAJ_NONE_FLAG;
	}
	if (i++ < samples) {
		parameter s = (ip == 'c') ? _cubic(T, _t)
					: (ip == 'q') ? _quintic(T, _t)
					: (ip == 't') ? _trapezoidal(pv, T, _t)
					: (ip == '1') ? _sCurve(pv, T, _t)
					: (ip == '2') ? _s2Curve(pv, T, _t)
					: _hsCurve(T, _t);
		Qc = isnanFix(addART(Q0, dotART(s.pos, dQ)));	// Qc = Q0 + s*dQ
		dQc = dotART(s.vel, dQ);			// dQc = ds*dQ
		ddQc = dotART(s.acell, dQ);			// ddQc = dds*dQ
		dddQc = dotART(s.jerk, dQ);			// dddQc = ddds*dQ
		_t += Tm;
		_time += Tm;
	}
	// unsigned long long tf = micros();
	// Serial.println(tf-t0);
}

/* HOME - Go to HOME
	This ROBOT function creates PTP trajectory from the current Robot Pose
	to HOME position
	HOME(t,pv)*/
void _HOME(double _T)
{
	T = _T/* 3.0 */; pv = 0.99999; ip = 'h';
	Q0 = Qc;
	Qf = Qhome;
	dQ = subART(Qf, Q0);

	i = 0;
	samples = (int) (_T/Tm + 1);

	trajFlags = TRAJ_PTP_FLAG;
}

void _WAIT(double _T)
{
	T = _T/* 3.0 */; pv = 0.99999; ip = '1';
	Q0 = Qc;
	Qf = Qc;
	dQ = subART(Qf, Q0);

	i = 0;
	samples = (int) (_T/Tm + 1);

	trajFlags = TRAJ_PTP_FLAG;
}

/* LIN - Rectilinear Motion
	This ROBOT command creates a 6-by-N-by-4 Matrix that contains a
	Rectilinear operational trajectory to a final robot pose. Sinusoidal
	jerk S curve interpolation as default.
	LIN(P,t,pv)*/
void _LIN(pose P, ...)
{
	T = 1.0; pv = 0.8; ip = 'h';
	va_list ap;
	va_start(ap, P);
	T = va_arg(ap, double);
	pv = va_arg(ap, double);
	ip = va_arg(ap, int);
	va_end(ap);
	/* Planning Path */
	// X0 = DK(Qc, 0, 0, 0);
	// X0 = subPOSE(X0, (pose){0.001,0.001,0.001,0.001,0.001,0.001});
	Xf = (pose){P.p, deg2rad(P.o)};
	// mth T0 = pose2mth(X0, 0, 0);
	mth T0 = dirKinematics(Qc);
	mth Tf = pose2mth(Xf, 0, 0);
	p0 = T0.p;
	R0 = T0.R;
	vec pf = Tf.p;
	mat Rf = Tf.R;
	dP = subV(pf, p0);
	vec u = zeros;
	prot pr = {{1, 1, 1}, 0};

#define ev(R1, R2) ((round(R1.r11*1e4) == round(R2.r11*1e4)) &&	\
					(round(R1.r12*1e4) == round(R2.r12*1e4)) &&	\
					(round(R1.r13*1e4) == round(R2.r13*1e4)) &&	\
					(round(R1.r21*1e4) == round(R2.r21*1e4)) &&	\
					(round(R1.r22*1e4) == round(R2.r22*1e4)) &&	\
					(round(R1.r23*1e4) == round(R2.r23*1e4)) &&	\
					(round(R1.r31*1e4) == round(R2.r31*1e4)) &&	\
					(round(R1.r32*1e4) == round(R2.r32*1e4)) &&	\
					(round(R1.r33*1e4) == round(R2.r33*1e4)))
	// if (ev(R0, Rf))
		pr = pairRot(dotM(transpose(R0), Rf));
	k = pr.k;
	theta = pr.theta;

	i = 0;
	samples = (int) (T/Tm + 1);

	trajFlags = TRAJ_LIN_FLAG;
}

void _LINMotionHandler()
{
	static double/*  t = 0.0, */ _t = 0.0;
	static art qant, qpant, qppant;

	// unsigned long long t0 = micros();

	if (i == 0) {
		_t = 0.0;
		qant = Qc;
		qpant = (art)zeros6;
		qppant = (art)zeros6;
		Q0 = Qc;
		X0 = Xf;
	}
	if (i == samples) {
		trajFlags = TRAJ_NONE_FLAG;
	}
	if (i++ < samples) {
		/* Timming law */
		parameter s = (ip == 'c') ? _cubic(T, _t)
					: (ip == 'q') ? _quintic(T, _t)
					: (ip == 't') ? _trapezoidal(pv, T, _t)
					: (ip == '1') ? _sCurve(pv, T, _t)
					: (ip == '2') ? _s2Curve(pv, T, _t)
					: _hsCurve(T, _t);
		/* Position */
		vec p = addV(p0, dot(s.pos, dP));
		mat R = dotM(R0, rot(k, theta * s.pos));
		art Qaux = ikSolveMTH((mth) {R, p}, Qc);
		// art Qaux = IK((pose){p, mat2rpy(R, 0)}, 0, 0, 1, 1, 0);
		// art Qaux = ikSolve((pose){p, mat2rpy(R, 0)}, Qc);
		Qc = isnanFix(Qaux);
		/* Velocity */
		dQc = dotART(1.0/Tm, subART(Qc, qant));
		/* Acceleration */
		ddQc = dotART(1.0/Tm, subART(dQc, qpant));
		/* Jerk */
		dddQc = dotART(1.0/Tm, subART(ddQc, qppant));
		/* Update joint variables */
		qant = Qc;
		qpant = dQc;
		qppant = ddQc;
		_t += Tm;
		_time += Tm;
	}
	// unsigned long long tf = micros();
	// Serial.println(tf-t0);
}

/* LIN - Rectilinear Motion
	This ROBOT command creates a 6-by-N-by-4 Matrix that contains a
	Circular operational trajectory to a final robot pose. Sinusoidal
	jerk S curve interpolation as default.
	CIRC(P,t,pv)*/
void _CIRC(pose Pa, pose Pf, ...)
{
	T = 1.0; pv = 0.8; ip = 'h';
	va_list ap;
	va_start(ap, Pf);
	T = va_arg(ap, double);
	pv = va_arg(ap, double);
	ip = va_arg(ap, int);
	va_end(ap);
	/* Planning Path */
	// X0 = subPOSE(X0, (pose){0.001,0.001,0.001,0.001,0.001,0.001});
	vec pa = Pa.p;
	X0 = DK(Qc, 0, 0, 0);
	Xf = (pose){Pf.p, deg2rad(Pf.o)};
	pose dX = subPOSE(Xf, X0);
	mth T0 = pose2mth(X0, 0, 0);
	mth Tf = pose2mth(Xf, 0, 0);
	p0 = T0.p;
	R0 = T0.R;
	vec pf = Tf.p;
	mat Rf = Tf.R;
	// Evaluating movement
	is_move_position = norm(subV(pf, p0));
	mat auxR = subM(Rf, R0);
	is_move_orientation = abs(auxR.r11+auxR.r12+auxR.r13+auxR.r21+auxR.r22+auxR.r23+auxR.r31+auxR.r32+auxR.r33) > 0.0;
	// Finding radius vectors
	if (is_move_position) {
		vec P = X0.p;
		vec Q = Pa.p;
		vec R = Pf.p;
		double norm_P = norm(P);
		double norm_Q = norm(Q);
		double norm_R = norm(R);
		vec PQ = subV(Q, P);
		vec PR = subV(R, P);
		n = cross(PQ,PR);
		mat A = { PQ.x, PQ.y, PQ.z,
				  PR.x, PR.y, PR.z,
				   n.x,  n.y, n.z};
		vec b = { (norm_Q*norm_Q - norm_P*norm_P)/2,
				  (norm_R*norm_R - norm_P*norm_P)/2,
				  dotV(n,P)};
		C_ = dotMV(inv(A),b);
		ri = subV(P, C_);
		vec ra = subV(Q, C_);
		vec rf = subV(R, C_);
		double ca1 = dotV(ri,ra)/(norm(ri)*norm(ra));
		double ca2 = dotV(ra,rf)/(norm(ra)*norm(rf));
		double ang1 = atan2(sqrt(1-ca1*ca1),ca1);
		double ang2 = atan2(sqrt(1-ca2*ca2),ca2);
		ang = ang1 + ang2;
	} else {
		n = { 1, 1, 1 };
		ang = 0;
	}
	// if (is_move_orientation) {
		prot pr = {{1, 1, 1}, 0};
		// if (norm(dX.o) > 0.0)
		if (is_move_orientation)
			pr = pairRot(dotM(transpose(R0), Rf));
		k = pr.k;
		theta = pr.theta;
		// Serial.print("Angle");Serial.println(theta);
	// } else {
	// 	k = { 1, 1, 1 };
	// 	theta = 0;
	// }
	n = unit(n);

	i = 0;
	samples = (int) (T/Tm + 1);

	trajFlags = TRAJ_CIRC_FLAG;
}

void _CIRCMotionHandler()
{
	static double/*  t = 0.0, */ _t = 0.0;
	static art qant, qpant, qppant;

	// unsigned long long t0 = micros();

	if (i == 0) {
		_t = 0.0;
		qant = Qc;
		qpant = (art)zeros6;
		qppant = (art)zeros6;
		Q0 = Qc;
		X0 = Xf;
	}
	if (i == samples) {
		trajFlags = TRAJ_NONE_FLAG;
	}
	if (i++ < samples) {
		/* Timming law */
		parameter s = (ip == 'c') ? _cubic(T, _t)
					: (ip == 'q') ? _quintic(T, _t)
					: (ip == 't') ? _trapezoidal(pv, T, _t)
					: (ip == '1') ? _sCurve(pv, T, _t)
					: (ip == '2') ? _s2Curve(pv, T, _t)
					: _hsCurve(T, _t);
		/* Position */
		vec p = p0, r;
		mat R = R0;
		if (is_move_position) {
			r = rotAxis(ri, n, ang * s.pos);
			p = addV(C_, r);
		}
		// if (is_move_orientation)
			R = dotM(R0, rot(k, theta * s.pos));
		art Qaux = ikSolve((pose){p, mat2rpy(R, 0)}, Qc);
		Qc = isnanFix(Qaux);
		/* Velocity */
		dQc = dotART(1.0/Tm, subART(Qc, qant));
		/* Acceleration */
		ddQc = dotART(1.0/Tm, subART(dQc, qpant));
		/* Jerk */
		dddQc = dotART(1.0/Tm, subART(ddQc, qppant));
		/* Update joint variables */
		qant = Qc;
		qpant = dQc;
		qppant = ddQc;
		_t += Tm;
		_time += Tm;
	}
	// unsigned long long tf = micros();
	// Serial.println(tf-t0);
}

bool someNanInJoint(art _Q) {
	return isnan(_Q.q1) || isnan(_Q.q2) || isnan(_Q.q3) ||
			isnan(_Q.q4) || isnan(_Q.q5) || isnan(_Q.q6);
}

art isnanFix(art _Q) {
	// art Q;
	// Q.q1 = (isnan(_Q.q1)) ? Qc.q1 : _Q.q1;
	// Q.q2 = (isnan(_Q.q2)) ? Qc.q2 : _Q.q2;
	// Q.q3 = (isnan(_Q.q3)) ? Qc.q3 : _Q.q3;
	// Q.q4 = (isnan(_Q.q4)) ? Qc.q4 : _Q.q4;
	// Q.q5 = (isnan(_Q.q5)) ? Qc.q5 : _Q.q5;
	// Q.q6 = (isnan(_Q.q6)) ? Qc.q6 : _Q.q6;
	// return Q;
	return (someNanInJoint(_Q)) ? Qc : _Q;
}

art Dynamics(art q, art _qp, art _qpp, vec Fe, vec Me) {
	double th[] = {q.q1, q.q2 + pi/2, q.q3, q.q4, q.q5, q.q6};
	double qp[] = {_qp.q1, _qp.q2, _qp.q3, _qp.q4, _qp.q5, _qp.q6};
	double qpp[] = {_qpp.q1, _qpp.q2, _qpp.q3, _qpp.q4, _qpp.q5, _qpp.q6};
	// mth A06 = dirKinematics(q);
	// vec Fe = dotMV(transpose(A06.R), F0);	// Force in end efector
	// vec Me = dotMV(transpose(A06.R), M0);	// Momentum in end efector
	/* ----------------------------------------------------------------- */
	// Dynamics
	// DH-parameters
	// double MDH[][4] = { {q.q1		, L1, L2,  pi/2},
	// 					   {q.q2 + pi/2 ,  0, L3,     0},
	// 					   {q.q3		,  0, L4,  pi/2},
	// 					   {q.q4		, L5,  0, -pi/2},
	// 					   {q.q5		,  0,  0,  pi/2},
	// 					   {q.q6		, L6,  0,     0}};
	mat R[7];
	R[6] = eye;
	vec r[6], w[7], wp[7], vp[7], ac[6];
	vec F[7], M[7];
	F[6] = Fe;
	M[6] = Me;
	double tau[6];
	// Forward Recursion
	for (int i = 0; i < 6; i++) {
		mth A = dh(th[i], d[i], a[i], alph[i]);	// Matrix (i-1)A(i)
		R[i] = A.R; // Matrix	(i-1)R(i)
		r[i] = {a[i], d[i]*sin(alph[i]), d[i]*cos(alph[i])};
		// Velocities and accelerations
		w[i+1]   = dotMV(transpose(R[i]),(addV(w[i], (vec){0, 0, qp[i]})));
        wp[i+1]  = addV(dotMV(transpose(R[i]),(addV(wp[i], (vec){0, 0, qpp[i]}))), cross(w[i], (vec){0, 0, qp[i]}));
        vp[i+1]  = addV(addV(dotMV(transpose(R[i]),vp[i]), cross(wp[i+1], r[i])), cross(w[i+1], cross(w[i+1], r[i])));
	}
	for (int i = 0; i < 6; i++)
		ac[i] = addV(addV(vp[i+1], cross(wp[i+1], c[i])), cross(w[i+1], cross(w[i+1], c[i])));
	// Backward Recursion
	for (int j = 0; j < 6; j++) {
		i = 6 - j - 1;
		F[i] = addV(dotMV(R[i+1],F[i+1]), dot(m[i],ac[i]));
		M[i] = addV(addV(addV(dotMV(R[i+1],(addV(M[i+1], cross(dotMV(transpose(R[i+1]),r[i]),F[i+1])))), cross(addV(r[i],c[i]),dot(m[i],ac[i]))), dotMV(I[i],wp[i+1])), cross(w[i+1],dotMV(I[i],w[i+1])));
		vec z = {R[i].r31, R[i].r32, R[i].r33};
		tau[i] = dotV(M[i],z);
	}
	return (art) {tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]};
}
