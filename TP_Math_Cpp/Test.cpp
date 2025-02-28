#include "Test.h"

#include "MathLib.h"

void testProdMat()
{
    Matrix m1(2, 3);
    m1[0][0] = 1;
    m1[0][1] = 2;
    m1[0][2] = 3;
    m1[1][0] = 4;
    m1[1][1] = 5;
    m1[1][2] = 6;
    MathLib::printMatrix(m1);
    Matrix m2(3, 2);
    m2[0][0] = 7;
    m2[0][1] = 8;
    m2[1][0] = 9;
    m2[1][1] = 10;
    m2[2][0] = 11;
    m2[2][1] = 12;
    MathLib::printMatrix(m2);
    Matrix m3 = m1 * m2;
    MathLib::printMatrix(m3);
}

void testInversedMatrix()
{
    Matrix m(3, 3);
    m[0][0] = 1;
    m[0][1] = -1;
    m[0][2] = 2;
    m[1][0] = 1;
    m[1][1] = 6;
    m[1][2] = 1;
    m[2][0] = 2;
    m[2][1] = 0;
    m[2][2] = -1;
    /*
    1 -1  2
    1  6  1
    2  0 -1
    */
    MathLib::printMatrix(m, "Base Matrix :");

    std::cout << "Determinant matrice : " << Matrix::deter(m) << '\n';

    Matrix comatrix = Matrix::com(m);
    MathLib::printMatrix(comatrix, "Comatrix :");

    Matrix transposed = Matrix::tran(m);
    MathLib::printMatrix(transposed, "Transposed :");

    Matrix inversed = Matrix::inverse(m);
    MathLib::printMatrix(inversed, "Inversed :");
}

void testTranslation()
{
    float m = 0.1f;
    float h = 0.1f;
    FVector F = FVector(2.f, 0.f, 0.f);
    FVector G = FVector(0.f, 0.f, 0.f);
    FVector v = FVector(1.f, 1.f, 1.f);
    std::cout << "Before translation \n";
    std::cout << "F: " << F.ToString() << '\n';
    std::cout << "G: " << G.ToString() << '\n';
    std::cout << "v: " << v.ToString() << '\n';
    DoubleVector result = MathLib::translation(m, h, F, G, v);
    result.print();
}

void testRotation()
{
    float h = 0.1f;
    FVector* F = new FVector[3]{ FVector(2.f, 0.f, 0.f), FVector(0.f, 3.f, 0.f), FVector(1.f, 1.f, 1.f) };
    FVector* A = new FVector[3]{ FVector(1.f, 2.f, 0.f), FVector(-1.f, -2.f, 1.f), FVector(0.f, 1.f, -1.f) };
    FVector G = FVector(0.f, 0.f, 0.f);
    Matrix I(3, 3);
    /*
    2 0 0
    0 3 0
    0 0 4
    */
    I[0][0] = 2;
    I[0][1] = 0;
    I[0][2] = 0;
    I[1][0] = 0;
    I[1][1] = 3;
    I[1][2] = 0;
    I[2][0] = 0;
    I[2][1] = 0;
    I[2][2] = 4;
    FVector teta = FVector(0.1f, 0.2f, 0.3f);
    FVector tetap = FVector(0.05f, -0.1f, 0.2f);

    DoubleVector result = MathLib::rotation(h, F, A, G, I, teta, tetap);
    result.print();

    delete[] F;
    delete[] A;
}