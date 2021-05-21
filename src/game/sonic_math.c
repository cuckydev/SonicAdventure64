#include "sonic_math.h"

#include <string.h>
#include "../engine/math_util.h"

//Vector
void njAddVector(Vector *a, const Vector *b)
{
	a->x = a->x + b->x;
	a->y = a->y + b->y;
	a->z = a->z + b->z;
}

f32 njInnerProduct(const Vector *a, const Vector *b)
{
	return a->z * b->z + a->y * b->y + a->x * b->x;
}

f32 njOuterProduct(const Vector *a, const Vector *b, Vector *c)
{
	f32 x, y, z, mag;
	x = b->z * a->y - b->y * a->z;
	y = b->x * a->z - b->z * a->x;
	z = b->y * a->x - b->x * a->y;
	c->x = x;
	c->y = y;
	c->z = z;
	mag = sqr(z) + sqr(y) + sqr(x);
	return sqrtf(mag);
}

//Matrix
static s32 sMatrixStackFlag;
static Matrix *sMatrixStack;
static Matrix *sMatrixStackPtr;
static u32 sMatrixStackCapacity;
static u32 sMatrixStackSize;

static const Matrix sUnitMatrix =
{
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f
};

void njInitMatrix(Matrix_ptr matrixStack, u32 size, s32 flag)
{
	//Initialize matrix stack
	sMatrixStackFlag = flag;
	sMatrixStack = (Matrix*)matrixStack;
	sMatrixStackPtr = sMatrixStack;
	sMatrixStackCapacity = size;
	sMatrixStackSize = 1;
	
	//Clear matrix stack
	memcpy(sMatrixStack, sUnitMatrix, sizeof(Matrix));
	(*sMatrixStack)[0] = -(*sMatrixStack)[0];
	(*sMatrixStack)[M11] = -(*sMatrixStack)[M11];
}

void njPushMatrix()
{
	//Push stack forward by 1 entry
	if (sMatrixStackSize < sMatrixStackCapacity)
	{
		memcpy(sMatrixStackPtr + 16, sMatrixStackPtr, sizeof(Matrix));
		sMatrixStackPtr += 16;
		++sMatrixStackSize;
	}
}

void njPopMatrix(u32 n)
{
	//Push back stack by n entries
	if (sMatrixStackSize != n)
	{
		sMatrixStackPtr -= n * 16;
		sMatrixStackSize -= n;
	}
}

void njUnitMatrix(Matrix_ptr matrix)
{
	//Get matrix to use (stack matrix if no matrix given)
	Matrix_ptr _matrix = matrix;
	if (matrix == NULL)
		_matrix = (Matrix_ptr)*sMatrixStackPtr;
	
	//Copy unit matrix to matrix
	memcpy(_matrix, sUnitMatrix, sizeof(Matrix));
}

void njRotateX(Matrix_ptr matrix, s16 angle)
{
	f32 sin, cos;
	f32 m10, m11, m12, m13;
	
	//Get matrix to use (stack matrix if no matrix given)
	Matrix_ptr _matrix = matrix;
	if (matrix == NULL)
		_matrix = (Matrix_ptr)*sMatrixStackPtr;
	
	//Calculate the sine and cosine of our angle
	sin = sins((u16)angle);
	cos = coss((u16)angle);
	
	//Apply rotation onto matrix
	m10 = _matrix[M10];
	m11 = _matrix[M11];
	m12 = _matrix[M12];
	m13 = _matrix[M13];
	
	_matrix[M10] = sin * _matrix[M20] + m10 * cos;
	_matrix[M11] = sin * _matrix[M21] + m11 * cos;
	_matrix[M12] = sin * _matrix[M22] + m12 * cos;
	_matrix[M13] = sin * _matrix[M23] + m13 * cos;
	_matrix[M20] = cos * _matrix[M20] - m10 * sin;
	_matrix[M21] = cos * _matrix[M21] - m11 * sin;
	_matrix[M22] = cos * _matrix[M22] - m12 * sin;
	_matrix[M23] = cos * _matrix[M23] - m13 * sin;
}

void njRotateY(Matrix_ptr matrix, s16 angle)
{
	f32 sin, cos;
	f32 m00, m01, m02, m03;
	
	//Get matrix to use (stack matrix if no matrix given)
	Matrix_ptr _matrix = matrix;
	if (matrix == NULL)
		_matrix = (Matrix_ptr)*sMatrixStackPtr;
	
	//Calculate the sine and cosine of our angle
	sin = sins((u16)angle);
	cos = coss((u16)angle);
	
	//Apply rotation onto matrix
	m00 = _matrix[M00];
	m01 = _matrix[M01];
	m02 = _matrix[M02];
	m03 = _matrix[M03];
	
	_matrix[M00] = m00 * cos - sin * _matrix[M20];
	_matrix[M01] = m01 * cos - sin * _matrix[M21];
	_matrix[M02] = m02 * cos - sin * _matrix[M22];
	_matrix[M03] = m03 * cos - sin * _matrix[M23];
	_matrix[M20] = cos * _matrix[M20] + m00 * sin;
	_matrix[M21] = cos * _matrix[M21] + m01 * sin;
	_matrix[M22] = cos * _matrix[M22] + m02 * sin;
	_matrix[M23] = cos * _matrix[M23] + m03 * sin;
}

void njRotateZ(Matrix_ptr matrix, s16 angle)
{
	f32 sin, cos;
	f32 m00, m01, m02, m03;
	
	//Get matrix to use (stack matrix if no matrix given)
	Matrix_ptr _matrix = matrix;
	if (matrix == NULL)
		_matrix = (Matrix_ptr)*sMatrixStackPtr;
	
	//Calculate the sine and cosine of our angle
	sin = sins((u16)angle);
	cos = coss((u16)angle);
	
	//Apply rotation onto matrix
	m00 = _matrix[M00];
	m01 = _matrix[M01];
	m02 = _matrix[M02];
	m03 = _matrix[M03];
	
	_matrix[M00] = m00 * cos + sin * _matrix[M10];
	_matrix[M01] = m01 * cos + sin * _matrix[M11];
	_matrix[M02] = m02 * cos + sin * _matrix[M12];
	_matrix[M03] = m03 * cos + sin * _matrix[M13];
	_matrix[M10] = cos * _matrix[M10] - m00 * sin;
	_matrix[M11] = cos * _matrix[M11] - m01 * sin;
	_matrix[M12] = cos * _matrix[M12] - m02 * sin;
	_matrix[M13] = cos * _matrix[M13] - m03 * sin;
}

void njCalcVector(Matrix_ptr matrix, const Vector *sourceVector, Vector *destinationVector)
{
	f32 sourceX, sourceY, sourceZ;
	f32 destinationX, destinationY, destinationZ;
	f32 magnitude, length;
	
	//Get matrix to use (stack matrix if no matrix given)
	Matrix_ptr _matrix = matrix;
	if (matrix == NULL)
		_matrix = (Matrix_ptr)*sMatrixStackPtr;
	
	//Get source coordinates
	sourceX = sourceVector->x;
	sourceY = sourceVector->y;
	sourceZ = sourceVector->z;
	
	//Set destination coordinates
	destinationVector->x = sourceZ * _matrix[M20] + sourceY * _matrix[M10] + sourceX * _matrix[M00];
	destinationVector->y = sourceZ * _matrix[M21] + sourceY * _matrix[M11] + sourceX * _matrix[M01];
	destinationVector->z = sourceZ * _matrix[M22] + sourceY * _matrix[M12] + sourceX * _matrix[M02];
	
	//Normalize if flag is set
	if (sMatrixStackFlag)
	{
		destinationX = destinationVector->x;
		destinationY = destinationVector->y;
		destinationZ = destinationVector->z;
		
		magnitude = sqr(destinationX) + sqr(destinationY) + sqr(destinationZ);
		length = 1.0f / sqrtf(magnitude);
		
		destinationVector->x = destinationX * length;
		destinationVector->y = destinationY * length;
		destinationVector->z = destinationZ * length;
	}
}
