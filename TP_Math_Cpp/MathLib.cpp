#include "MathLib.h"


void MathLib::printMatrix(const Matrix& m, const char* text)
{
	std::cout << text << '\n' << m.ToString();
}

// Solve equation f + f' * h
float MathLib::solve1(float f, float fp, float h)
{
	return f + fp * h;
}

/**
* @param m : mass
* @param h : time step
* @param F : sum forces
* @param G : inertia center
* @param v : speed
*/
DoubleVector MathLib::translation(float m, float h, const FVector& F, const FVector& G, const FVector& v)
{
	FVector accel = F / m;
	FVector newV = accel * h + v;
	FVector newG = newV * h + G;
	return { newG, newV };
}

/**
* @param h : time step
* @param F : list of forces
* @param A : list of application points
* @param G : inertia center
* @param I : inertia matrix
* @param teta : angle
* @param tetap : angular speed
*/
DoubleVector MathLib::rotation(float h, FVector* F, FVector* A, const FVector& G, const Matrix& I, const FVector& teta, const FVector& tetap)
{
	FVector newG = FVector::Zero();
	for (int i = 0; i < 3; ++i) {
		newG = newG + FVector::moment(F[i], A[i], G);
	}

	// Angular acceleration
	FVector omegaDot = newG * Matrix::inverse(I);
	FVector newTetap = tetap + omegaDot * h;
	FVector newTeta = teta + newTetap * h;

	return { newTeta, newTetap };
}
