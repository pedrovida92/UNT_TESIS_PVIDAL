#ifndef SPATIAL_H_
#define SPATIAL_H_

#include <stddef.h>
#include <stdbool.h>
#include "linalg.c"

#define pi	3.14159265358979323846

typedef struct {
	vec k;
	double theta;
} prot;

vec rect2cyl(vec);
vec cyl2rect(vec);
vec rect2sph(vec);
vec sph2rect(vec);
vec cyl2sph(vec);
vec sph2cyl(vec);

mat rotx(double);
mat roty(double);
mat rotz(double);

mat wuw2mat(vec);
mat wvw2mat(vec);
mat rpy2mat(vec);

vec mat2wuw(mat,bool);
vec mat2wvw(mat,bool);
vec mat2rpy(mat,bool);

mat rot(vec,double);
prot pairRot(mat);
vec rotAxis(vec,vec,double);


#endif /* SPATIAL_H_ */