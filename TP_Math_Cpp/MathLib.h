#pragma once

#include <vector>

#include "Matrix.h"
#include "FVector.h"
#include "StructHeader.h"

namespace MathLib
{
	void printMatrix(const Matrix& m, const char* text = "Matrix :");
	float solve1(float f, float fp, float h);
	DoubleVector translation(float m, float h, const FVector& F, const FVector& G, const FVector& v);
    DoubleVector rotation(float h, const std::vector<FVector>& F, const std::vector<FVector>& A, const FVector& G, const Matrix& I, const FVector& teta, const FVector& tetap);
	FVector centre_inert(const std::vector<FVector>& L);
	Matrix matrice_inert(const std::vector<FVector>& L, float m);
	Matrix deplace_matrix(const Matrix& I, float m, const FVector& O, const FVector& A);
}
