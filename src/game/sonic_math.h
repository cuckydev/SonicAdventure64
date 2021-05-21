#ifndef _SONIC_NJREIMP_H
#define _SONIC_NJREIMP_H

//Ninja library reimplementation
#include <ultra64.h>

//Vector
typedef struct
{
	f32 x, y, z;
} Vector;

void njAddVector(Vector *a, const Vector *b);
f32 njInnerProduct(const Vector *a, const Vector *b);
f32 njOuterProduct(const Vector *a, const Vector *b, Vector *c);

//Quaternion
typedef struct
{
	f32 x, y, z, w;
} Quaternion;

//Matrix
typedef f32 Matrix[16];
typedef f32 *Matrix_ptr;

enum MAT_INDEX
{
    M00, M01, M02, M03,
    M10, M11, M12, M13,
    M20, M21, M22, M23,
    M30, M31, M32, M33
};

void njInitMatrix(Matrix_ptr matrixStack, u32 size, s32 flag);
void njPushMatrix();
void njPopMatrix(u32 n);
void njUnitMatrix(Matrix_ptr matrix);
void njRotateX(Matrix_ptr matrix, s16 angle);
void njRotateY(Matrix_ptr matrix, s16 angle);
void njRotateZ(Matrix_ptr matrix, s16 angle);
void njCalcVector(Matrix_ptr matrix, const Vector *sourceVector, Vector *destinationVector);

#endif
