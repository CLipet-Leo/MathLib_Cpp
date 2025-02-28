#include "MathLib.h"

#include <iostream>
#include <string>

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
DoubleVector MathLib::rotation(float h, const std::vector<FVector>& F, const std::vector<FVector>& A, const FVector& G, const Matrix& I, const FVector& teta, const FVector& tetap)
{
	if (F.size() != A.size())
		throw std::invalid_argument("F and A must have the same size");
	
	FVector newG = FVector::Zero();
	for (int i = 0; i < F.size(); ++i)
		newG = newG + FVector::moment(F[i], A[i], G);

	// Angular acceleration
	FVector omegaDot = newG * Matrix::inverse(I);
	FVector newTetap = tetap + omegaDot * h;
	FVector newTeta = teta + newTetap * h;

	return { newTeta, newTetap };
}

/**
 * 
 * @param L : List of points
 * @return : Center of inertia
 */
FVector MathLib::centre_inert(const std::vector<FVector>& L)
{
	FVector G = FVector::Zero();
	for (const auto& i : L)
		G = G + i;
	return G / L.size();
}

/**
 * 
 * @param L List of points
 * @param m Total mass
 * @return The inertia matrix
 */
Matrix MathLib::matrice_inert(const std::vector<FVector>& L, float m)
{
	float massPerPoint = m / L.size();
	float A = 0, B = 0, C = 0, D = 0, E = 0, F = 0;
	for (const auto& i : L)
	{
		A += (pow(i.getY(), 2.f) + pow(i.getZ(), 2.f)) * massPerPoint;
		B += (pow(i.getX(), 2.f) + pow(i.getZ(), 2.f)) * massPerPoint;
		C += (pow(i.getX(), 2.f) + pow(i.getY(), 2.f)) * massPerPoint;
		D += i.getY() * i.getZ() * massPerPoint;
		E += i.getX() * i.getZ() * massPerPoint;
		F += i.getX() * i.getY() * massPerPoint;
	}
	Matrix I(3, 3);
	I[0][0] = A;
	I[0][1] = -F;
	I[0][2] = -E;
	I[1][0] = -F;
	I[1][1] = B;
	I[1][2] = -D;
	I[2][0] = -E;
	I[2][1] = -D;
	I[2][2] = C;
	return I;
}

Matrix MathLib::deplace_matrix(const Matrix& I, float m, const FVector& O, const FVector& A)
{
	if (I.getRows() != 3 || I.getCols() != 3)
		throw std::invalid_argument("Matrix must be 3x3");
	const FVector OA = FVector::distance(O, A);
	Matrix IOA(3, 3);
	IOA[0][0] = m * (pow(OA.getY(), 2.f) + pow(OA.getZ(), 2.f));
	IOA[0][1] = -m * OA.getX() * OA.getY();
	IOA[0][2] = -m * OA.getX() * OA.getZ();
	IOA[1][0] = -m * OA.getX() * OA.getY();
	IOA[1][1] = m * (pow(OA.getX(), 2.f) + pow(OA.getZ(), 2.f));
	IOA[1][2] = -m * OA.getY() * OA.getZ();
	IOA[2][0] = -m * OA.getX() * OA.getZ();
	IOA[2][1] = -m * OA.getY() * OA.getZ();
	IOA[2][2] = m * (pow(OA.getX(), 2.f) + pow(OA.getY(), 2.f));
	return I + IOA;
}
