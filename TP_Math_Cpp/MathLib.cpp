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

// Factorial function
unsigned long long MathLib::factoriel(unsigned int n)
{
	unsigned long long result = 1;
	for (unsigned int i = 2; i <= n; ++i)
		result *= i;
	return result;
}

/**
 * Cosinus function
 * @param x : number
 * @param n : iterations
 * @return : cosinus of x
 */
double MathLib::cosinus(double x, int n)
{
	double cos_x = 1.0; // First term of the Taylor series
	double power_x = 1.0; // x^0 = 1
	double factorial = 1.0; // 0! = 1
	int sign = 1; // (-1)^i
	for (int i = 1; i < n; ++i)
	{
		power_x *= x * x; // x^(2*i)
		factorial *= (2 * i - 1) * (2 * i); // (2*i)!
		sign = -sign; // Switch between + and -
		cos_x += sign * (power_x / factorial);
	}
	return cos_x;
}

/**
 * Sinus function
 * @param x : number
 * @param n : iterations
 * @return : sinus of x
 */
double MathLib::sinus(double x, int n)
{
	double sin_x = x; // First term of the Taylor series
	double power_x = x; // x^1 = x
	double factorial = 1.0; // 1! = 1
	int sign = 1; // (-1)^i
	for (int i = 1; i < n; ++i)
	{
		power_x *= x * x; // x^(2*i+1)
		factorial *= 2 * i * (2 * i + 1); // (2*i+1)!
		sign = -sign; // Switch between + and -
		sin_x += sign * (power_x / factorial);
	}
	return sin_x;
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

/**
 * Create a matrix of points in a geometrical shape
 * @param n : number of points
 * @param a : length
 * @param b : width
 * @param c : height
 * @param A0 : first point
 * @return : Matrix of points with 3 rows representing the coordinates X, Y and Z
 */
Matrix MathLib::pave_plein(unsigned int n, float a, float b, float c, const FVector3& A0)
{
	const int iteration = static_cast<int>(pow(n, 1.f / 3.f));
	const float dx = a / (iteration - 1);
	const float dy = b / (iteration - 1);
	const float dz = c / (iteration - 1);
	Matrix M(3, n);
	for (int i = 0; i < iteration; ++i)
		for (int j = 0; j < iteration; ++j)
			for (int k = 0; k < iteration; ++k)
			{
				const int index = i * iteration * iteration + j * iteration + k;
				M[0][index] = A0.getX() + i * dx;
				M[1][index] = A0.getY() + j * dy;
				M[2][index] = A0.getZ() + k * dz;
			}
	return M;
}

/**
 * Create a matrix of points in a circle
 * @param R : radius
 * @param A0 : first point
 * @param n : number of points
 * @return : Matrix of points with 3 rows representing the coordinates X, Y and Z
 */
Matrix MathLib::cercle_plein(float R, const FVector3& A0, int n)
{
	int n_r = static_cast<int>(std::sqrt(n)); // Subdivisions for the radius
	int n_t = std::max(1, n / n_r); // Subdivisions for the angle
	Matrix M(3, n);
	int index = 0;
	// Add the first point once
	M[0][index] = A0.getX();
	M[1][index] = A0.getY();
	M[2][index] = A0.getZ();
	++index;
	for (int i = 1; i < n_r + 1; ++i) 
	{
		float r = R * (static_cast<float>(i) / n_r); // Progressive radius
		for (int j = 0; j < n_t; ++j) 
		{
			double theta = 2 * M_PI * j / n_t; // Uniformly distributed angle
			M[0][index] = static_cast<float>(A0.getX() + r * cosinus(theta));
			M[1][index] = static_cast<float>(A0.getY() + r * sinus(theta));
			M[2][index] = A0.getZ();
			++index;

			// Security check
			if (index >= n) break;
		}
		if (index >= n) break;
	}
	return M;
}
