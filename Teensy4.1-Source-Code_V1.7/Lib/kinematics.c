#include "kinematics.h"

/* addART - Addition of two joint configurations */
art addART(art Q1, art Q2)
{
	Q1.q1 += Q2.q1;
	Q1.q2 += Q2.q2;
	Q1.q3 += Q2.q3;
	Q1.q4 += Q2.q4;
	Q1.q5 += Q2.q5;
	Q1.q6 += Q2.q6;
	return Q1;
}

/* subART - Substraction of two joint configurations */
art subART(art Q1, art Q2)
{
	Q1.q1 -= Q2.q1;
	Q1.q2 -= Q2.q2;
	Q1.q3 -= Q2.q3;
	Q1.q4 -= Q2.q4;
	Q1.q5 -= Q2.q5;
	Q1.q6 -= Q2.q6;
	return Q1;
}

/* dotART - Scalar multiplication of a joint configurations */
art dotART(double s, art Q) {
	Q.q1 *= s;
	Q.q2 *= s;
	Q.q3 *= s;
	Q.q4 *= s;
	Q.q5 *= s;
	Q.q6 *= s;
	return Q;
}

/* addPOSE - Addition of two poses */
pose addPOSE(pose X1, pose X2)
{
	X1.p = addV(X1.p, X2.p);
	X1.o = addV(X1.o, X2.o);
	return X1;
}

/* subPOSE - Substraction of two poses */
pose subPOSE(pose X1, pose X2)
{
	X1.p = subV(X1.p, X2.p);
	X1.o = subV(X1.o, X2.o);
	return X1;
}

/* subART - Norm of joint configuration vector */
double normART(art Q1)
{
	return sqrt(Q1.q1 * Q1.q1 + Q1.q2 * Q1.q2 + Q1.q3 * Q1.q3 + Q1.q4 * Q1.q4 + Q1.q5 * Q1.q5 + Q1.q6 * Q1.q6);
}

/* subART - Norm of pose vector */
double normPOSE(pose X1)
{
	return sqrt(norm2(X1.p) + norm2(X1.o));
}

/* dotMTH - Homogeneous Transformation Matrix Multiplication */
mth dotMTH(mth m1, mth m2)
{
	mth temp;
	temp.R = dotM(m1.R, m2.R);
	temp.p = addV(m1.p, dotMV(m1.R, m2.p));
	return temp;
}

/* dh - Denavit-Hartenberg Homogeneous Transformation Matrix
	This ROBOT function creates a Homogeneous Transformation Matrix given
	Denavit-Hartenberg Parameters for Direct Kinematics.
	T = dh(theta,d,a,alpha) */
mth dh(double theta, double d, double a, double alpha)
{
	double ct = cos(theta);
	double st = sin(theta);
    double ca = cos(alpha);
	double sa = sin(alpha);
    mth T = { (mat) { ct, -ca*st,  sa*st,
          			  st,  ca*ct, -sa*ct,
           			   0,     sa,     ca },
			  (vec) { a*ct, a*st, d } };
	return T;
}

/* dk - Direct Kinematic
	This ROBOT function creates a HTM that contains position and
	orientation of the End Effector respect to Base. A06.
	A06 = dirKinematics(q) */
mth dirKinematics(art q)
{
	mth A01 = dh(q.q1		, L1, L2,  pi/2);
	mth A12 = dh(q.q2 + pi/2,  0, L3,     0);
	mth A23 = dh(q.q3		,  0, L4,  pi/2);
	mth A34 = dh(q.q4		, L5,  0, -pi/2);
	mth A45 = dh(q.q5		,  0,  0,  pi/2);
	mth A56 = dh(q.q6		, L6,  0,     0);
	return dotMTH( dotMTH( dotMTH( dotMTH( dotMTH( A01, A12 ), A23 ), A34 ), A45 ), A56 );
}

/* ik - Inverse Kinematic
	This ROBOT function returns the joint configuration for a pose of robot
	in Operational Space.
	q = invKinematics(T,s,e,w) */
art invKinematics(mth T, bool ss, bool ee, bool ww)
{
	art q;
	/* ******* First three Degrees of Freedom ******* */
    vec P = T.p;   // End Effector Position
    vec z6 = {T.R.r13, T.R.r23, T.R.r33};  // Z6 vector
    // Kinematic decoupling
    vec Pm = addV(P, dot(-L6,z6));
    // Robot Configuration
    int s = ss ? 1 : -1;
    int e = ee ? 1 : -1;
    int w = ww ? 1 : -1;
    // Zero evaluation of wrist position
    double x = (int)(Pm.x*1e7) ? Pm.x : 0.0;
    double y = (int)(Pm.y*1e7) ? Pm.y : 0.0;
    double z = (int)(Pm.z*1e7) ? Pm.z : 0.0;
    // Required computation
    double R = s*sqrt(x*x + y*y);
    double r = sqrt(pow(R-L2,2.0) + pow(z-L1,2.0));
    double d = sqrt(L4*L4 + L5*L5);
    double phi = atan2(L4,L5);
    double cx = (r*r - d*d - L3*L3)/(2*d*L3);
    double sx = e*sqrt(1 - cx*cx);
    // Auxiliar q3
    double qx = atan2(sx,cx);
    // Auxiliars q2
    double beta = -atan2(R-L2, z-L1);
    double alfa =  atan2(d*sx, L3+d*cx);
    // Arm joints
    q.q3 = pi/2 - phi - qx;
    q.q2 = alfa + beta;
    q.q1 = atan2(s*y,s*x);
    /* ******* Last three Degrees of Freedom ******* */
    // Required computation
    mth A01 = dh(       q.q1, L1, L2, pi/2);
    mth A12 = dh(q.q2 + pi/2,  0, L3,    0);
    mth A23 = dh(       q.q3,  0, L4, pi/2);
    mat R03 = dotM( dotM( A01.R, A12.R ), A23.R );
    mat R06 = T.R;
    // {S6} respect to {S0} Rotation Matrix
    mat R36 = dotM(transpose(R03), R06);
    // Required computation
    double c5 =  R36.r33;
    double s5 =  w*sqrt(1 - c5*c5);
    double s4 =  R36.r23/s5;
    double c4 =  R36.r13/s5;
    double s6 =  R36.r32/s5;
    double c6 = -R36.r31/s5;
    // Wrist joints
    q.q4 = atan2(s4,c4);
    q.q5 = atan2(s5,c5);
    q.q6 = atan2(s6,c6);
    // Joint vector
    return q;
}

/* DKs - Direct Kinematic (Q -> X)
	This ROBOT function creates a Operational Vector given a Joint Vector.
	op_p:	opcion de posicion
	op_o:	opcion de orientacion
	X = DKs(Q,np,no,sol) */
pose DK(art Q, short op_p, short op_o, bool sol)
{
	pose X;
	mth T = dirKinematics(Q);
	
	switch (op_p) {
		case 0:
			X.p = T.p;
			break;
		case 1:
			X.p = rect2cyl(T.p);
			break;
		case 2:
			X.p = rect2sph(T.p);
			break;
		default:
			X.p = zeros;
	}
	
	switch (op_o) {
		case 0:
			X.o = mat2rpy(T.R,sol);
			break;
		case 1:
			X.o = mat2wuw(T.R,sol);
			break;
		case 2:
			X.o = mat2wvw(T.R,sol);
			break;
		default:
			X.o = zeros;
	}
	return X;
}

mth pose2mth(pose X, short op_p, short op_o)
{
	mth T;
	switch (op_p) {
		case 0:
			T.p = X.p;
			break;
		case 1:
			T.p = cyl2rect(X.p);
			break;
		case 2:
			T.p = sph2rect(X.p);
			break;
		default:
			T.p = zeros;
	}
	switch (op_o) {
		case 0:
			T.R = rpy2mat(X.o);
			break;
		case 1:
			T.R = wuw2mat(X.o);
			break;
		case 2:
			T.R = wvw2mat(X.o);
			break;
		default:
			T.R = eye;
	}
	return T;
}

/* IKs - Inverse Kinematic (X -> Q)
	This ROBOT function creates a Joint Vector given a Operational Vector.
	Q = IKs(X,op_p,op_o,s,e,w,mode) */
art IK(pose X, short op_p, short op_o, bool s, bool e, bool w)
{
	mth T = pose2mth(X, op_p, op_o);
	// Serial.printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
    //             T.R.r11, T.R.r12, T.R.r13, T.p.x,
	// 			T.R.r21, T.R.r22, T.R.r23, T.p.y,
	// 			T.R.r31, T.R.r32, T.R.r33, T.p.z);
	art Q = invKinematics(T,s,e,w);
	// Serial.printf("articulaciones:  %f, %f, %f, %f, %f, %f\n",
    //             Q.q1,
    //             Q.q2,
    //             Q.q3,
	// 			Q.q4,
    //             Q.q5,
    //             Q.q6);
	// art Q = invKinematics(dotMTH(T, invMTH(TCP)),s,e,w);
	return Q;
}

mth invMTH(mth T) {
	return (mth) { (mat) transpose(T.R),
				   (vec) { dotV({-T.R.r11, -T.R.r21, -T.R.r31}, T.p),
						   dotV({-T.R.r12, -T.R.r22, -T.R.r32}, T.p),
					 	   dotV({-T.R.r13, -T.R.r23, -T.R.r33}, T.p)}};
}

/*mat6 J(q):
    # Sin and Cos for calculation
    s1 = sin(q[0]);     c1 = cos(q[0]);     s23 = sin(q[1]+q[2])
    s2 = sin(q[1]);     c2 = cos(q[1]);     c23 = cos(q[1]+q[2])
    s4 = sin(q[3]);     c4 = cos(q[3])
    s5 = sin(q[4]);     c5 = cos(q[4])
    # Recycling variables
    a1 =   c4*s23;  a2 = c5*c23;    a3 = s4*s23;    a4 = s5*c23
    a5 =   s4*c23;  a6 = c5*s23;    a8 = s5*s23
    a9 =   L2 - L3*s2 - L4*s23 + L5*c23
    a10 =  L4*c23 + L5*s23 + L6*(a6 + c4*a4)
    a11 = -L4*s23 + L5*c23 + L6*(a2 - s5*a1)
    a7 =   L3*c2+a10;
    b1 =  c1*s4 + s1*a1
    b2 =  s1*s4 - c1*a1
    b3 =  s1*c4 + c1*a3
    b4 =  c1*c4 - s1*a3
    # Jacobian Matrix
    return array([[L6*(s5*b1-s1*a2)-s1*a9,     -c1*a7, -c1*a10,  L6*s5*b3,  L6*(c5*b2-c1*a4),            0],
                  [L6*(s5*b2+c1*a2)+c1*a9,     -s1*a7, -s1*a10, -L6*s5*b4, -L6*(c5*b1+s1*a4),            0],
                  [                     0, -L3*s2+a11,     a11, -L6*s4*a4,    -L6*(a8-c4*a2),            0],
                  [                     0,         s1,      s1,    c1*c23,                b3,  s5*b2+c1*a2],
                  [                     0,        -c1,     -c1,    s1*c23,               -b4, -s5*b1+s1*a2],
                  [                     1,          0,       0,       s23,               -a5,     a6+c4*a4]])

/* printMTH:	imprime una matriz de transformacion homogenea 4x4 */
void printMTH(mth m)
{
	Serial.printf("\t[%7.4f, %7.4f, %7.4f, %9.4f\n", m.R.r11, m.R.r12, m.R.r13, m.p.x);
	Serial.printf("\t %7.4f, %7.4f, %7.4f, %9.4f\n", m.R.r21, m.R.r22, m.R.r23, m.p.y);
	Serial.printf("\t %7.4f, %7.4f, %7.4f, %9.4f\n", m.R.r31, m.R.r32, m.R.r33, m.p.z);
	Serial.printf("\t %7.0f, %7.0f, %7.0f, %9.0f]" ,     0.0,     0.0,     0.0,   1.0);
}

/* printlnMTH:	imprime una MTH con salto de linea*/
void printlnMTH(mth m)
{
	printMTH(m);
	printf("\n");
}

/* printART:	imprime un vector articular 6D */
void printART(art q)
{
	printf("\t[%7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f]\n",
			q.q1, q.q2, q.q3, q.q4, q.q5, q.q6);
}

/* printlnART:	imprime un vector articular 6D con salto de linea */
void printlnART(art q)
{
	printART(q);
	printf("\n");
}

/* printPOSE:	imprime un vector Operacional 6D */
void printPOSE(pose X)
{
	printf("\t[%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f]\n",
			X.p.x, X.p.y, X.p.z, X.o.x, X.o.y, X.o.z);
}

/* printlnPOSE:	imprime un vector Operacional 6D con salto de linea */
void printlnPOSE(pose X)
{
	printPOSE(X);
	printf("\n");
}
