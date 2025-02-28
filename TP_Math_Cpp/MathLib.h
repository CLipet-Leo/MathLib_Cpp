#pragma once

#include <iostream>
#include <string>
#include "Matrix.h"
#include "FVector.h"
#include "StructHeader.h"

namespace MathLib
{
	void printMatrix(const Matrix& m, const char* text = "Matrix :");
	float solve1(float f, float fp, float h);
	DoubleVector translation(float m, float h, const FVector& F, const FVector& G, const FVector& v);
    DoubleVector rotation(float h, FVector* F, FVector* A, const FVector& G, const Matrix& I, const FVector& teta, const FVector& tetap);
}
