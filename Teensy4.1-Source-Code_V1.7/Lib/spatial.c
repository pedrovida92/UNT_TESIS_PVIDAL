#include <math.h>
#include "spatial.h"

/* rect2cyl - 3-D Rectangular to Cylindrical Coordinates Transformation
	This ROBOT function transform a tree-dimensional position vector from
	Rectangular to Cylindrical Coordinates.
	V = rect2cyl(P)     ->      V = [theta,rho,z] */
vec rect2cyl(vec P)
{
	double theta = atan2(P.y, P.x);
	double rho = sqrt(P.x*P.x + P.y*P.y);
	vec V = {theta, rho, P.z};
	return V;
}

/* cyl2rect - 3-D Cylindrical to Rectangular Coordinates Transformation
	This ROBOT function transform a tree-dimensional position vector from
	Cylindrical to Rectangular Coordinates.
	V = cyl2rect(P)     ->      P = [theta,rho,z] */
vec cyl2rect(vec P)
{
	double theta = P.x;
	double rho = P.y;
	double x = rho*cos(theta);
	double y = rho*sin(theta);
	vec V = {x,y,P.z};
	return V;
}

/* rect2sph - 3-D Rectangular to Spherical Coordinates Transformation
	This ROBOT function transform a tree-dimensional position vector from
	Rectangular to Spherical Coordinates.
	V = rect2sph(P)     ->      V = [r,phi,theta] */
vec rect2sph(vec P)
{
	double r = norm(P);
	double phi = acos(P.z/r);
	double theta = atan2(P.y, P.x);
	vec V = {r, phi, theta};
	return V;
}

/* sph2rect - 3-D Spherical to Rectangular Coordinates Transformation
	This ROBOT function transform a tree-dimensional position vector from
	Spherical to Rectangular Coordinates.
	V = sph2rect(P)     ->      P = [r,phi,theta] */
vec sph2rect(vec P)
{
	double r = P.x;
	double phi = P.y;
	double theta = P.z;
	double x = r*sin(phi)*cos(theta);
	double y = r*sin(phi)*sin(theta);
	double z = r*cos(phi);
	vec V = {x, y, z};
	return V;
}

/* cyl2sph - 3-D Cylindrical to Spherical Coordinates Transformation
	This ROBOT function transform a tree-dimensional position vector from
	Cylindrical to Spherical Coordinates.
	V = cyl2sph(P)      ->      P = [theta,rho,z]
	                            V = [r,phi,theta] */
vec cyl2sph(vec P)
{
	double theta = P.x;
	double rho = P.y;
	double z = P.z;
	double r = sqrt(rho*rho + z*z);
	double phi = atan2(rho, z);
	vec V = {r, phi, theta};
	return V;
}

/* sph2cyl - 3-D Spherical to Cylindrical Coordinates Transformation
	This ROBOT function transform a tree-dimensional position vector from
	Spherical to Cylindrical Coordinates.
	V = sph2cyl(P)      ->      P = [r,phi,theta]
	                            V = [theta,rho,z] */
vec sph2cyl(vec P)
{
	double r = P.x;
    double phi = P.y;
    double theta = P.z;
    double rho = r*sin(phi);
    double z = r*cos(phi);
    vec V = {theta, rho, z};
    return V;
}

/* rotx - Rotation matrix for rotations around x-axis
	This ROBOT function creates a 3-by-3 Rotation Matrix that represents
	a rotation around x-axis by ang in a three-dimensional space.
	R = rotx(ang) */
mat rotx(double ang)
{
	double c = cos(ang);
    double s = sin(ang);
    mat R = {1, 0, 0,
			 0, c,-s,
			 0, s, c};
	return R;
}

/* roty - Rotation matrix for rotations around y-axis
	This ROBOT function creates a 3-by-3 Rotation Matrix that represents
	a rotation around y-axis by ang in a three-dimensional space.
	R = roty(ang) */
mat roty(double ang)
{
	double c = cos(ang);
    double s = sin(ang);
    mat R = { c, 0, s,
			  0, 1, 0,
			 -s, 0, c};
	return R;
}

/* rotz - Rotation matrix for rotations around z-axis
	This ROBOT function creates a 3-by-3 Rotation Matrix that represents
	a rotation around z-axis by ang in a three-dimensional space.
	R = rotz(ang) */
mat rotz(double ang)
{
	double c = cos(ang);
    double s = sin(ang);
    mat R = {c,-s, 0,
			 s, c, 0,
			 0, 0, 1};
	return R;
}

/* wuw2mat -  WUW Euler Angles to Rotation Matrix
	This ROBOT function creates a 3-by-3 Rotation Matrix that represents
	a rotation conrresponding to wuw Euler Angles.
	R = wuw2mat(wuw) */
mat wuw2mat(vec wuw)
{
	mat R = dotM( dotM( rotz(wuw.x), rotx(wuw.y) ), rotz(wuw.z) );
	return R;
}

/* wvw2mat - WVW Euler Angles to Rotation Matrix
	This ROBOT function creates a 3-by-3 Rotation Matrix that represents
	a rotation conrresponding to wvw Euler Angles.
	R = wvw2mat(wvw) */
mat wvw2mat(vec wvw)
{
	mat R = dotM( dotM( rotz(wvw.x), roty(wvw.y) ), rotz(wvw.z) );
	return R;
}

/*rpy2mat - XYZ Euler Angles to Rotation Matrix
	This ROBOT function creates a 3-by-3 Rotation Matrix that represents
	a rotation conrresponding to XYZ Euler Angles.
	R = rpy2mat(rpy) */
mat rpy2mat(vec rpy)
{
	mat R = dotM( dotM( rotz(rpy.x), roty(rpy.y) ), rotx(rpy.z) );
	return R;
}

/* mat2wuw - Rotation Matrix to WUW Euler Angles
	This ROBOT function returns two WUW Euler Angles solutions given a
	3-by-3 Rotation Matrix.
	V = mat2wuw(R,sol) */
vec mat2wuw(mat R, bool sol)
{
	vec V;
	
	double sb = sqrt(R.r31*R.r31 + R.r32*R.r32);
    double cb = R.r33;
    
    if (!sol) {
    	/* First solution:   0 < b < pi */
		V.z = atan2(R.r31, R.r32);
		V.y = atan2(sb,cb);
		V.x = atan2(R.r13,-R.r23);
	} else {
		/* Second solution:  -pi < b < 0 */
		V.y = atan2(-sb,cb);
		V.z = atan2(-R.r31,-R.r32);
		V.x = atan2(-R.r13, R.r23);
	}
	return V;
}

/* mat2wvw - Rotation Matrix to WVW Euler Angles
	This ROBOT function returns two WVW Euler Angles solutions given a
	3-by-3 Rotation Matrix.
	V = mat2wvw(R,sol) */
vec mat2wvw(mat R, bool sol)
{
	vec V;
	
	double sb = sqrt(R.r31*R.r31 + R.r32*R.r32);
    double cb = R.r33;
    
    if (!sol) {
    	/* First solution:   0 < b < pi */
		V.z = atan2(R.r32,-R.r31);
		V.y = atan2(sb,cb);
		V.x = atan2(R.r23, R.r13);
	} else {
		/* Second solution:  -pi < b < 0 */
		V.y = atan2(-sb,cb);
		V.z = atan2(-R.r32, R.r31);
		V.x = atan2(-R.r23,-R.r13);
	}
	return V;
}

/* mat2rpy - Rotation Matrix to XYZ Euler Angles
	This ROBOT function returns two XYZ Euler Angles solutions given a
	3-by-3 Rotation Matrix.
	V = mat2rpy(R,sol) */
vec mat2rpy(mat R, bool sol)
{
	vec V;
	
	double cp = sqrt(R.r32*R.r32 + R.r33*R.r33);
    double sp = -R.r31;
    
    if (!sol) {
    	/* First solution:   -pi/2 < p < pi/2 */
		V.x = atan2(R.r21, R.r11);
		V.y = atan2(sp,cp);
		V.z = atan2(R.r32, R.r33);
	} else {
		/* Second solution:  pi/2 < p < 3*pi/2 */
		V.x = atan2(-R.r21,-R.r11);
		V.y = atan2(sp,-cp);
		V.z = atan2(-R.r32,-R.r33);
	}
	return V;
}

/* rot - Rotation matrix for rotation around k-axis
	This ROBOT function creates a 3-by-3 Rotation Matrix that represents
	a rotation around a unit vector by theta in a three-dimensional space.
	R = rot(k,theta) */
mat rot(vec k, double theta)
{
	vec r = unit(k);
    double rx = r.x;
    double ry = r.y;
    double rz = r.z;
    double s = sin(theta);
    double c = cos(theta);
    mat R = {     c+rx*rx*(1-c),-rz*s+rx*ry*(1-c), ry*s+rx*rz*(1-c),
    		   rz*s+rx*ry*(1-c),    c+ry*ry*(1-c),-rx*s+ry*rz*(1-c),
    		  -ry*s+rx*rz*(1-c), rx*s+ry*rz*(1-c),    c+rz*rz*(1-c)};
    return R;
}

/* pairRot - Rotation Pair of a Rotation Matrix
	This ROBOT function creates a Rotation Pair given a Rotation Matrix
	that represents Rigid Body Rotation.
	{k,theta} = pairRot(R) */
prot pairRot(mat R)
{
	prot pRot;
	
	double theta = acos((R.r11 + R.r22 + R.r33 - 1)/2);
	pRot.theta = theta;
	
	if (theta) {
		pRot.k = dot(1/(2*sin(theta)), (vec) {R.r32-R.r23,
                    						  R.r13-R.r31,
                    						  R.r21-R.r12});
	} else
		pRot.k = (vec) {1,1,1};
	return pRot;
}

/* rotAxis - Rotation Pair of a 3-D Vector
	This ROBOT function rotate a three-dimensional vector given a Roatation
	Pair.
	V = rotAxis(P,k,theta) */
vec rotAxis(vec P, vec k, double ang)
{
	// k = unit(k);
    double s = sin(ang);
    double c = cos(ang);
    vec V = addV( addV( dot(c,P), dot(s,cross(k,P)) ), dot(dotV(k,P)*(1-c),k) );
    return V;
}
