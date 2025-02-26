#pragma once

#include <iostream>
#include <string>
#include "FVector.h"
#include "StructHeader.h"

namespace MathLib
{
	Matrix createMatrix(int rows, int cols);
	void deleteMatrix(Matrix& m);
	void printMatrix(const Matrix& m, const char* text = "Matrix :");
	Matrix prodmat(const Matrix& m1, const Matrix& m2);
	Matrix prodValueMat(const Matrix& m, float value);
	float deter(const Matrix& m);
	Matrix subMatrix(const Matrix& m, int row, int col);
	Matrix com(const Matrix& m);
	Matrix tran(const Matrix& m);
	Matrix inverse(const Matrix& m);
	float solve1(float f, float fp, float h);
	DoubleVector translation(float m, float h, const FVector& F, const FVector& G, const FVector& v);
    DoubleVector rotation(float h, FVector* F, FVector* A, const FVector& G, const Matrix& I, const FVector& teta, const FVector& tetap);
};
