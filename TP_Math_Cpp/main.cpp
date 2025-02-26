#include <iostream>
#include <string>

#include "MathLib.h"

using namespace MathLib;

static void testProdMat()
{
	Matrix m1 = createMatrix(2, 3);
	m1.data[0][0] = 1;
	m1.data[0][1] = 2;
	m1.data[0][2] = 3;
	m1.data[1][0] = 4;
	m1.data[1][1] = 5;
	m1.data[1][2] = 6;
	printMatrix(m1);
	Matrix m2 = createMatrix(3, 2);
	m2.data[0][0] = 7;
	m2.data[0][1] = 8;
	m2.data[1][0] = 9;
	m2.data[1][1] = 10;
	m2.data[2][0] = 11;
	m2.data[2][1] = 12;
	printMatrix(m2);
	Matrix m3 = prodmat(m1, m2);
	printMatrix(m3);
	deleteMatrix(m1);
	deleteMatrix(m2);
	deleteMatrix(m3);
}

static void testInversedMatrix()
{
	Matrix m = createMatrix(3, 3);
	m.data[0][0] = 1;
	m.data[0][1] = -1;
	m.data[0][2] = 2;
	m.data[1][0] = 1;
	m.data[1][1] = 6;
	m.data[1][2] = 1;
	m.data[2][0] = 2;
	m.data[2][1] = 0;
	m.data[2][2] = -1;
	/*
	1 -1  2
	1  6  1
	2  0 -1
	*/
	printMatrix(m, "Base Matrix :");

	std::cout << "Determinant matrice : " << deter(m) << std::endl;

	Matrix comatrix = com(m);
	printMatrix(comatrix, "Comatrix :");

	Matrix transposed = tran(m);
	printMatrix(transposed, "Transposed :");

	Matrix inversed = inverse(m);
	printMatrix(inversed, "Inversed :");

	deleteMatrix(m);
	deleteMatrix(comatrix);
	deleteMatrix(transposed);
	deleteMatrix(inversed);
}

static void testTranslation()
{
	float m = 0.1f;
	float h = 0.1f;
	FVector F = FVector(2.f, 0.f, 0.f);
	FVector G = FVector(0.f, 0.f, 0.f);
	FVector v = FVector(1.f, 1.f, 1.f);
	std::cout << "Before translation" << std::endl;
	std::cout << "F: " << F.ToString() << std::endl;
	std::cout << "G: " << G.ToString() << std::endl;
	std::cout << "v: " << v.ToString() << std::endl;
	DoubleVector result = translation(m, h, F, G, v);
	result.print();
}

static void testRotation()
{
	float h = 0.1f;
	FVector* F = new FVector[3]{ FVector(2.f, 0.f, 0.f), FVector(0.f, 3.f, 0.f), FVector(1.f, 1.f, 1.f) };
	FVector* A = new FVector[3]{ FVector(1.f, 2.f, 0.f), FVector(-1.f, -2.f, 1.f), FVector(0.f, 1.f, -1.f) };
	FVector G = FVector(0.f, 0.f, 0.f);
	Matrix I = createMatrix(3, 3);
	/*
	2 0 0
	0 3 0
	0 0 4
	*/
	I.data[0][0] = 2;
	I.data[0][1] = 0;
	I.data[0][2] = 0;
	I.data[1][0] = 0;
	I.data[1][1] = 3;
	I.data[1][2] = 0;
	I.data[2][0] = 0;
	I.data[2][1] = 0;
	I.data[2][2] = 4;
	FVector teta = FVector(0.1f, 0.2f, 0.3f);
	FVector tetap = FVector(0.05f, -0.1f, 0.2f);

	DoubleVector result = rotation(h, F, A, G, I, teta, tetap);
	result.print();

	delete[] F;
	delete[] A;
}

int main()
{
	//testProdMat();
	//testInversedMatrix();
	//testTranslation();
	testRotation();

	return 0;
}
