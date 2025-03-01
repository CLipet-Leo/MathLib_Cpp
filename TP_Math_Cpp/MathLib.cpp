#include "MathLib.h"

#include <iostream>
#include <string>

/**
 * Function to print a matrix
 * @param m : Matrix to print
 * @param text : Optional text before the matrix
 */
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
 * Translate an object with a mass, a time step, a sum of forces, an inertia center and a speed
* @param m : mass
* @param h : time step
* @param F : sum forces
* @param G : inertia center
* @param v : speed
* @return : New position and speed
*/
DoubleVector3 MathLib::translation(float m, float h, const FVector3& F, const FVector3& G, const FVector3& v)
{
	FVector3 accel = F / m;
	FVector3 newV = accel * h + v;
	FVector3 newG = newV * h + G;
	return { newG, newV };
}

/**
 * Rotate an object with a mass, a time step, a list of forces, a list of application points, an inertia center, an inertia matrix, an angle and an angular speed
* @param h : time step
* @param F : list of forces
* @param A : list of application points
* @param G : inertia center
* @param I : inertia matrix
* @param teta : angle
* @param tetap : angular speed
* @return : New angle and angular speed
*/
DoubleVector3 MathLib::rotation(float h, const std::vector<FVector3>& F, const std::vector<FVector3>& A, const FVector3& G, const Matrix& I, const FVector3& teta, const FVector3& tetap)
{
	if (F.size() != A.size())
		throw std::invalid_argument("F and A must have the same size");
	
	FVector3 newG = FVector3::Zero();
	for (int i = 0; i < F.size(); ++i)
		newG += FVector3::moment(F[i], A[i], G);

	// Angular acceleration
	FVector3 omegaDot = newG * Matrix::inverse(I);
	FVector3 newTetap = tetap + omegaDot * h;
	FVector3 newTeta = teta + newTetap * h;

	return { newTeta, newTetap };
}

/**
 * Calculate the center of inertia for a given list of points
 * @param L : List of points
 * @return : Center of inertia
 */
FVector3 MathLib::centre_inert(const std::vector<FVector3>& L)
{
	FVector3 G = FVector3::Zero();
	for (const auto& i : L)
		G += i;
	return G / static_cast<float>(L.size());
}

/**
 * Calculate the inertia matrix for a given list of points and mass
 * @param L List of points
 * @param m Total mass
 * @return The inertia matrix
 */
Matrix MathLib::matrice_inert(const std::vector<FVector3>& L, float m)
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

/**
 * Move a matrix
 * @param I : Matrix to deplace
 * @param m : mass
 * @param O : point O
 * @param A : point A
 * @return : Deplaced matrix
 */
Matrix MathLib::deplace_matrix(const Matrix& I, float m, const FVector3& O, const FVector3& A)
{
	if (I.getRows() != 3 || I.getCols() != 3)
		throw std::invalid_argument("Matrix must be 3x3");
	const FVector3 OA = FVector3::distance(O, A);
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
