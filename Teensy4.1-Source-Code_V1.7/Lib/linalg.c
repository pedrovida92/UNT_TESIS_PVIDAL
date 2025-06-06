#include "linalg.h"

/* addV:	devuelve la suma de dos vectores */
vec addV(vec v1, vec v2)
{
	v1.x += v2.x;
	v1.y += v2.y;
	v1.z += v2.z;
	return v1;
}

/* subV:	devuelve la resta de dos vectores */
vec subV(vec v1, vec v2)
{
	v1.x -= v2.x;
	v1.y -= v2.y;
	v1.z -= v2.z;
	return v1;
}

/* dotV:	devuelve el producto escalar de vectores */
double dotV(vec v1, vec v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/* dot:	devuelve el producto escalar de vector con escalar */
vec dot(double a, vec v)
{
	v.x *= a;
	v.y *= a;
	v.z *= a;
	return v;
}

/* cross:	devuelve el producto vectorial */
vec cross(vec u, vec v)
{
	vec w = {u.y*v.z - u.z*v.y,
			 u.z*v.x - u.x*v.z,
			 u.x*v.y - u.y*v.x};
	return w;
}

/* addM:	devuelve la suma de dos matrices 3x3 */
mat addM(mat m1, mat m2)
{
	m1.r11 += m2.r11;	m1.r12 += m2.r12;	m1.r13 += m2.r13;
	m1.r21 += m2.r21;	m1.r22 += m2.r22;	m1.r23 += m2.r23;
	m1.r31 += m2.r31;	m1.r32 += m2.r32;	m1.r33 += m2.r33;
	return m1;
}

/* subM:	devuelve la resta de dos matrices 3x3 */
mat subM(mat m1, mat m2)
{
	m1.r11 -= m2.r11;	m1.r12 -= m2.r12;	m1.r13 -= m2.r13;
	m1.r21 -= m2.r21;	m1.r22 -= m2.r22;	m1.r23 -= m2.r23;
	m1.r31 -= m2.r31;	m1.r32 -= m2.r32;	m1.r33 -= m2.r33;
	return m1;
}

/* dotM:	devuelve el producto escalar de matrices 3x3 */
mat dotM(mat m1, mat m2)
{
	mat temp;
	temp.r11 = m1.r11*m2.r11 + m1.r12*m2.r21 + m1.r13*m2.r31;
	temp.r12 = m1.r11*m2.r12 + m1.r12*m2.r22 + m1.r13*m2.r32;
	temp.r13 = m1.r11*m2.r13 + m1.r12*m2.r23 + m1.r13*m2.r33;
	temp.r21 = m1.r21*m2.r11 + m1.r22*m2.r21 + m1.r23*m2.r31;
	temp.r22 = m1.r21*m2.r12 + m1.r22*m2.r22 + m1.r23*m2.r32;
	temp.r23 = m1.r21*m2.r13 + m1.r22*m2.r23 + m1.r23*m2.r33;
	temp.r31 = m1.r31*m2.r11 + m1.r32*m2.r21 + m1.r33*m2.r31;
	temp.r32 = m1.r31*m2.r12 + m1.r32*m2.r22 + m1.r33*m2.r32;
	temp.r33 = m1.r31*m2.r13 + m1.r32*m2.r23 + m1.r33*m2.r33;
	return temp;
}

/* transpose:	Matriz transpueta de 3x3 */
mat transpose(mat m)
{
	mat temp;
	temp.r11 = m.r11;	temp.r12 = m.r21;	temp.r13 = m.r31;
	temp.r21 = m.r12;	temp.r22 = m.r22;	temp.r23 = m.r32;
	temp.r31 = m.r13;	temp.r32 = m.r23;	temp.r33 = m.r33;
	return temp;
	
}

/* dotMV:	Multiplicacion matriz 3x3 por vector 3D */
vec dotMV(mat m1, vec v)
{
	vec temp;
	temp.x = m1.r11*v.x + m1.r12*v.y + m1.r13*v.z;
	temp.y = m1.r21*v.x + m1.r22*v.y + m1.r23*v.z;
	temp.z = m1.r31*v.x + m1.r32*v.y + m1.r33*v.z;
	return temp;
}

/* det:	determinante de una matriz 3x3 */
double det(mat m)
{
	double temp = m.r11*m.r22*m.r33
				+ m.r12*m.r23*m.r31
				+ m.r13*m.r21*m.r32
				- m.r13*m.r22*m.r31
				- m.r11*m.r23*m.r32
				- m.r12*m.r21*m.r33;
	return temp;
}

/* adj:	adjunta de una matriz 3x3 */
mat adj(mat m)
{
	mat temp;
	temp.r11 =  m.r22*m.r33 - m.r23*m.r32;
	temp.r12 = -m.r21*m.r33 + m.r23*m.r31;
	temp.r13 =  m.r21*m.r32 - m.r22*m.r31;

	temp.r21 = -m.r12*m.r33 + m.r13*m.r32;
	temp.r22 =  m.r11*m.r33 - m.r13*m.r31;
	temp.r23 = -m.r11*m.r32 + m.r12*m.r31;

	temp.r31 =  m.r12*m.r23 - m.r13*m.r22;
	temp.r32 = -m.r11*m.r23 + m.r13*m.r21;
	temp.r33 =  m.r11*m.r22 - m.r12*m.r21;
	return temp;
}

/* inv:	inversa de una matriz 3x3 */
mat inv(mat m)
{
	double a = 1/det(m);
	mat temp = transpose(adj(m));
	temp.r11 *= a;
	temp.r12 *= a;
	temp.r13 *= a;
	temp.r21 *= a;
	temp.r22 *= a;
	temp.r23 *= a;
	temp.r31 *= a;
	temp.r32 *= a;
	temp.r33 *= a;
	return temp;
}

/* norm:	norma de vector 3D */
double norm(vec v)
{
	return sqrt(norm2(v));
}

/* norm2:	norma 2 de vector 3D */
double norm2(vec v)
{
	return dotV(v,v);
}

/* unit:	vector unitario de vector 3D */
vec unit(vec v)
{
	double norm_v = norm(v);
	v.x /= norm_v;
	v.y /= norm_v;
	v.z /= norm_v;
	return v;
}

/* printV:	imprime un vector 3D */
void printV(vec v)
{
	printf("\t[%8.4f, %8.4f, %8.4f]",v.x, v.y, v.z);
}

/* printlnV:	imprime un vector 3D con salto de linea*/
void printlnV(vec v)
{
	printV(v);
	printf("\n");
}

/* printM:	imprime un una matriz 3x3 */
void printM(mat m)
{
	printf("\t[%8.4f, %8.4f, %8.4f\n", m.r11, m.r12, m.r13);
	printf("\t %8.4f, %8.4f, %8.4f\n", m.r21, m.r22, m.r23);
	printf("\t %8.4f, %8.4f, %8.4f]" , m.r31, m.r32, m.r33);
}

/* printlnM:	imprime una matriz 3x3 con salto de linea*/
void printlnM(mat m)
{
	printM(m);
	printf("\n");
}
