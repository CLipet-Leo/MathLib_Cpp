#include "MathLib.h"


Matrix MathLib::createMatrix(int rows, int cols) {
	Matrix m;
	m.rows = rows;
	m.cols = cols;
	m.data = new float* [rows];
	for (int i = 0; i < rows; i++) {
		m.data[i] = new float[cols];
	}
	return m;
}

void MathLib::deleteMatrix(Matrix& m) {
	for (int i = 0; i < m.rows; i++) {
		delete[] m.data[i];
	}
	delete[] m.data;
}

void MathLib::printMatrix(const Matrix& m, const char* text) {
	std::cout << text << '\n';
	for (int i = 0; i < m.rows; i++) {
		for (int j = 0; j < m.cols; j++) {
			std::cout << m.data[i][j] << " ";
		}
		std::cout << '\n';
	}
}

Matrix MathLib::prodmat(const Matrix& m1, const Matrix& m2)
{
	if (m1.cols != m2.rows)
		throw std::runtime_error("Matrix dimensions are not compatible for multiplication.");
	Matrix newMat = createMatrix(m1.rows, m2.cols);
	for (int i = 0; i < m1.rows; i++)
	{
		for (int j = 0; j < m2.cols; j++)
		{
			newMat.data[i][j] = 0;
			for (int k = 0; k < m1.cols; k++)
			{
				newMat.data[i][j] += m1.data[i][k] * m2.data[k][j];
			}
		}
	}
	return newMat;
}

Matrix MathLib::prodValueMat(const Matrix& m, float value)
{
	Matrix newMat = createMatrix(m.rows, m.cols);
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			newMat.data[i][j] = m.data[i][j] * value;
		}
	}
	return newMat;
}

Matrix MathLib::subMatrix(const Matrix& m, int row, int col)
{
	Matrix newMat = createMatrix(m.rows - 1, m.cols - 1);
	int sub_i = 0;
	int sub_j = 0;
	for (int i = 0; i < m.rows; i++)
	{
		if (i == row)
			continue;
		for (int j = 0; j < m.cols; j++)
		{
			if (j == col)
				continue;
			newMat.data[sub_i][sub_j] = m.data[i][j];
			sub_j++;
		}
		sub_i++;
		sub_j = 0;
	}
	return newMat;
}

float MathLib::deter(const Matrix& m)
{
	float determinant = 0;
	if (m.rows == 1 && m.cols == 1)
		determinant = m.data[0][0];
	else
	{
		int j = 0;
		for (int i = 0; i < m.rows; i++)
			determinant += pow(-1, i + j) * m.data[i][j] * deter(subMatrix(m, i, j));
	}
	return determinant;
}

Matrix MathLib::com(const Matrix& m)
{
	Matrix comatrix = createMatrix(m.rows, m.cols);
	if (deter(m) == 0)
		throw std::runtime_error("Matrix determinant is zero, cannot compute comatrix.");
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			comatrix.data[i][j] = pow(-1, i + j) * deter(subMatrix(m, i, j));
		}
	}
	return comatrix;
}

Matrix MathLib::tran(const Matrix& m)
{
	Matrix transpose = createMatrix(m.cols, m.rows);
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			transpose.data[j][i] = m.data[i][j];
		}
	}
	return transpose;
}

Matrix MathLib::inverse(const Matrix& m)
{
	return prodValueMat(tran(com(m)), 1 / deter(m));
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
	FVector accel = FVector::divValue(F, m);
	FVector newV = FVector::prodValue(accel, h) + v;
	FVector newG = FVector::prodValue(newV, h) + G;
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
	FVector omegaDot = FVector::prodMatrix(newG, MathLib::inverse(I));
	FVector newTetap = tetap + FVector::prodValue(omegaDot, h);
	FVector newTeta = teta + FVector::prodValue(newTetap, h);

	return { newTeta, newTetap };
}
