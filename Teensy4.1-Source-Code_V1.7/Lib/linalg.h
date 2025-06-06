#ifndef LINALG_H_
#define LINALG_H_

// #include <stdio.h>
// #include <LibPrintf.h>
#include <math.h>

typedef struct {
	double x;
	double y;
	double z;
} vec;

typedef struct {
	double r11, r12, r13;
	double r21, r22, r23;
	double r31, r32, r33;
} mat;

/* Prototipo de funciones */
vec addV(vec,vec);
vec subV(vec,vec);
vec dot(double,vec);
double dotV(vec,vec);
vec cross(vec,vec);
mat addM(mat,mat);
mat subM(mat,mat);
mat dotM(mat,mat);
mat transpose(mat);
vec dotMV(mat,vec);

double det(mat);
mat adj(mat);
mat inv(mat);

double norm(vec);
double norm2(vec);
vec unit(vec);

void printV(vec);
void printlnV(vec);
void printM(mat);
void printlnM(mat);


#endif /* LINALG_H_ */
