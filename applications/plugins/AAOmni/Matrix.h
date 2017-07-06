//minimal functions for high performance
//compatible with c
#ifndef _AA_MATRIX4X4_H_
#define _AA_MATRIX4X4_H_
typedef struct Matrix4x4d
{
	double data[4][4];
} Matrix4x4d;

void MatrixMultiply(Matrix4x4d* a,Matrix4x4d* b,Matrix4x4d* res);
void MatrixInitialize(Matrix4x4d* a);
#endif