#pragma once

#include "Matrix.h"
#include "FVector3.h"

#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace MathLib
{
	void printMatrix(const Matrix& m, const char* text = "Matrix :");
	float solve1(float f, float fp, float h);
	unsigned long long factoriel(unsigned int n);
	double cosinus(double x, int n = 10);
	double sinus(double x, int n = 10);
	DoubleVector3 translation(float m, float h, const FVector3& F, const FVector3& G, const FVector3& v);
    DoubleVector3 rotation(float h, const std::vector<FVector3>& F, const std::vector<FVector3>& A, const FVector3& G, const Matrix& I, const FVector3& teta, const FVector3& tetap);
	FVector3 centre_inert(const std::vector<FVector3>& L);
	Matrix matrice_inert(const std::vector<FVector3>& L, float m);
	Matrix deplace_matrix(const Matrix& I, float m, const FVector3& O, const FVector3& A);
	Matrix pave_plein(unsigned int n,float a,float b,float c,const FVector3& A0);
	Matrix cercle_plein(float R,const FVector3& A0, int n = 100);
}
