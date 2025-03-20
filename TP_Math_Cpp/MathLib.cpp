#include "MathLib.h"

#include <algorithm>
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

double MathLib::truncate(double value, int precision)
{
	double factor = std::pow(10.0, precision);
	return std::floor(value * factor) / factor;
}

double MathLib::roundToPrecision(double value, int precision)
{
	double factor = std::pow(10.0f, precision);
	return std::round(value * factor) / factor;
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
	double term = 1.0;
	int sign = -1;  
	for (int i = 1; i < n; ++i)
	{
		term *= (x * x) / ((2 * i - 1) * (2 * i));
		cos_x += sign * term;
		sign = -sign;
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
	double term = x;  
	int sign = -1;  
	for (int i = 1; i < n; ++i)
	{
		term *= (x * x) / ((2 * i) * (2 * i + 1));
		sin_x += sign * term;
		sign = -sign;
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
	
	FVector3 torque = FVector3::Zero();
	for (size_t i = 0; i < F.size(); ++i)
		torque = torque + FVector3::moment(F[i], A[i], G);

	// Calculate angular acceleration
	Matrix I_inv = Matrix::inverse(I);
	FVector3 angularAcc = I_inv * torque;

	// Update angular speed and angle
	FVector3 newAngularVel = tetap + angularAcc * h;
	FVector3 newAngles = teta + newAngularVel * h;

	return { newAngles, newAngularVel };
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
 * Move an inertia matrix
 * @param I : Matrix to move
 * @param m : mass
 * @param O : Origin point
 * @param A : Arrival point
 * @return : Moved matrix to point A
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

Matrix MathLib::rotation_forme(Matrix W, const FVector3& G, const FVector3& teta)
{
	// Extract the angles
	float thetaX = teta.getX();
	float thetaY = teta.getY();
	float thetaZ = teta.getZ();

	// Build the rotation matrices
	float cX = static_cast<float>(cosinus(thetaX));
	float sX = static_cast<float>(sinus(thetaX));
	float cY = static_cast<float>(cosinus(thetaY));
	float sY = static_cast<float>(sinus(thetaY));
	float cZ = static_cast<float>(cosinus(thetaZ));
	float sZ = static_cast<float>(sinus(thetaZ));
	Matrix Rx = {
		{1, 0, 0},
		{0, cX, -sX},
		{0, sX,  cX}
	};
	Matrix Ry = {
		{ cY, 0, sY},
		{  0, 1,  0},
		{-sY, 0, cY}
	};
	Matrix Rz = {
		{ cZ, -sZ, 0},
		{ sZ,  cZ, 0},
		{  0,   0, 1}
	};

	// Combined rotation matrix
	Matrix R = Rz * Ry * Rx;

	// Apply the rotation to each point
	for (int i = 0; i < W.getCols(); i++)
	{
		FVector3 P(W[0][i], W[1][i], W[2][i]);
		FVector3 P_rotated = R * (P - G) + G;

		W[0][i] = P_rotated.getX();
		W[1][i] = P_rotated.getY();
		W[2][i] = P_rotated.getZ();
	}

	return W;
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
	// Calculate the number of iterations for each dimension
	const int iteration = static_cast<int>(std::cbrt(n));
	// Calculate the total number of points generated
	const int totalPoints = static_cast<int>(std::pow(iteration, 3));
	// Calculate the intervals between points in each dimension
	const float dx = a / (iteration - 1);
	const float dy = b / (iteration - 1);
	const float dz = c / (iteration - 1);
	Matrix M(3, totalPoints);
	int index = 0;
	for (int i = 0; i < iteration && index < n; ++i)
		for (int j = 0; j < iteration && index < n; ++j)
			for (int k = 0; k < iteration && index < n; ++k)
			{
				// Calculate the coordinates of each points
				M[0][index] = A0.getX() + i * dx;
				M[1][index] = A0.getY() + j * dy;
				M[2][index] = A0.getZ() + k * dz;
				++index;
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
	n = std::max(n, 3); // Sécurisation
	int n_r = n;  // `n_r` est le nombre d'anneaux
	int n_total = n_r * n + 1;  // Nombre total de points (y compris centre)

	Matrix M(3, n_total);
	int index = 0;

	// Ajouter le centre
	M[0][index] = A0.getX();
	M[1][index] = A0.getY();
	M[2][index] = A0.getZ();
	++index;

	// Générer les anneaux
	for (int i = 1; i <= n_r; ++i)
	{
		double r = R * (static_cast<double>(i) / static_cast<double>(n_r)); // Rayon progressif
		for (int j = 0; j < n; ++j)
		{
			double theta = 2.0 * M_PI * static_cast<double>(j) / static_cast<double>(n);
			M[0][index] = A0.getX() + r * cosinus(theta);
			M[1][index] = A0.getY() + r * sinus(theta);
			M[2][index] = A0.getZ();
			++index;
		}
	}

	return M;
}

/**
 * Create a matrix of points in a cylinder
 * @param R : radius
 * @param h : height
 * @param A0 : first point
 * @param n : number of points per radius in each circle
 * @param s_h : number of height slices
 * @return : Matrix of points with 3 rows representing the coordinates X, Y and Z
 */
Matrix MathLib::cylindre_plein(float R, float h, const FVector3& A0, int n, int s_h)
{
	h = std::max<float>(h, 1);
	n = std::max(n, 3);
	s_h = std::max(s_h, 2);

	int n_cercles = h * s_h;  // Total number of circles
	int n_r = n;  // Number of points per radius in each circle
	int n_total = n_cercles * (n_r * n + 1);  // Total number of points (including center)

	Matrix M(3, n_total);
	int index = 0;

	for (int i = 0; i < n_cercles; ++i)
	{
		float z = A0.getZ() + static_cast<float>(i) / s_h;  // Height of the circle

		// Generate a circle at height `z`
		Matrix cercle = cercle_plein(R, FVector3(A0.getX(), A0.getY(), z), n);

		// Add the circle to the matrix
		for (int j = 0; j < cercle.getCols(); ++j)
		{
			M[0][index] = cercle[0][j];
			M[1][index] = cercle[1][j];
			M[2][index] = cercle[2][j];
			++index;
		}
	}

	return M;
}

/**
 * Move a solid with a mass, an inertia matrix, a center of gravity, a linear speed, an angular vector and an angular speed
 * @param W : Solid matrix
 * @param m : mass
 * @param I : Inertia matrix
 * @param G : Center of gravity
 * @param v : Linear speed
 * @param teta : Angular vector
 * @param tetap : Angular speed
 * @param F : List of forces
 * @param A : List of application points
 * @param h : Time step
 * @return : New solid matrix, center of gravity, linear speed, angular vector and angular speed
 */
MovementResult MathLib::mouvement(Matrix W, float m, Matrix I, FVector3 G, FVector3 v, FVector3 teta, FVector3 tetap,
	std::vector<std::vector<FVector3>> F, std::vector<std::vector<FVector3>> A, float h)
{
	FVector3 totalForce = FVector3::Zero();
	for (const auto& forceList : F)
		for (const auto& f : forceList)
			totalForce = totalForce + f;
	
	DoubleVector3 trans = translation(m, h, totalForce, G, v);
	FVector3 newG = trans.v1; // Nouveau centre d'inertie
	FVector3 newV = trans.v2; // Nouvelle vitesse linéaire
	
	// Prepare the forces and points for the rotation calculation
	std::vector<FVector3> forcesFlat, pointsFlat;
	for (const auto& liste : F)
	    forcesFlat.insert(forcesFlat.end(), liste.begin(), liste.end());
	for (const auto& liste : A)
	    pointsFlat.insert(pointsFlat.end(), liste.begin(), liste.end());
	
	// Calculate the new angles and angular speed
	DoubleVector3 rot = rotation(h, forcesFlat, pointsFlat, G, I, teta, tetap);
	FVector3 newAngles = rot.v1;
	FVector3 newAngularVel = rot.v2;

	// Update the inertia matrix
	Matrix newI = deplace_matrix(I, m, G, newG);

	// Apply the rotation to the solid
	Matrix newW = rotation_forme(W, newG, newAngles);

	return { newW, newG, newV, newAngles, newAngularVel };
}

std::vector<Matrix> MathLib::trace_mouvements(Matrix W, float m, Matrix I, FVector3 G, FVector3 v, FVector3 teta,
	FVector3 tetap, const std::vector<std::vector<FVector3>>& F, const std::vector<std::vector<FVector3>>& A, float h,
	float t, int n)
{
	// Vector to store the snapshots
	std::vector<Matrix> snapshots;
	snapshots.reserve(n);

	h = t / static_cast<float>(n);

	for (int i = 0; i < n; i++)
	{
		// Call the movement function
		MovementResult result = mouvement(W, m, I, G, v, teta, tetap, F, A, h);

		// Stock the new matrix in the vector
		snapshots.push_back(result.newW);

		W     = result.newW;
		G     = result.newG;
		v     = result.newV;
		teta  = result.newTeta;
		tetap = result.newTetap;
	}

	return snapshots;
}
