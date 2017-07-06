//minimal functions for high performance
//compatible with c
#ifndef _AA_MATRIX4X4_H_
#define _AA_MATRIX4X4_H_

typedef struct Matrix4x4d
{
	double data[4][4];
} Matrix4x4d;

void MatrixMultiply(Matrix4x4d* a,Matrix4x4d* b,Matrix4x4d* res)
{
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
		{
			res->data[i][j]=0;
			for(int k=0;k<4;k++)
				res->data[i][j]+=a->data[i][k]*b->data[k][j];
		}
}

void MatrixInitialize(Matrix4x4d* a)
{
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			a->data[i][j]=0;
}
#endif