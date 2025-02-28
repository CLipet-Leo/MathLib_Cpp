#include "MathLib.h"

static void testProdMat()
{
    Matrix m1 = MathLib::createMatrix(2, 3);
    m1.data[0][0] = 1;
    m1.data[0][1] = 2;
    m1.data[0][2] = 3;
    m1.data[1][0] = 4;
    m1.data[1][1] = 5;
    m1.data[1][2] = 6;
    MathLib::printMatrix(m1);
    Matrix m2 = MathLib::createMatrix(3, 2);
    m2.data[0][0] = 7;
    m2.data[0][1] = 8;
    m2.data[1][0] = 9;
    m2.data[1][1] = 10;
    m2.data[2][0] = 11;
    m2.data[2][1] = 12;
    MathLib::printMatrix(m2);
    Matrix m3 = MathLib::prodmat(m1, m2);
    MathLib::printMatrix(m3);
    MathLib::deleteMatrix(m1);
    MathLib::deleteMatrix(m2);
    MathLib::deleteMatrix(m3);
}

static void testInversedMatrix()
{
    Matrix m = MathLib::createMatrix(3, 3);
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
    MathLib::printMatrix(m, "Base Matrix :");

    std::cout << "Determinant matrice : " << MathLib::deter(m) << '\n';

    Matrix comatrix = MathLib::com(m);
    MathLib::printMatrix(comatrix, "Comatrix :");

    Matrix transposed = MathLib::tran(m);
    MathLib::printMatrix(transposed, "Transposed :");

    Matrix inversed = MathLib::inverse(m);
    MathLib::printMatrix(inversed, "Inversed :");

    MathLib::deleteMatrix(m);
    MathLib::deleteMatrix(comatrix);
    MathLib::deleteMatrix(transposed);
    MathLib::deleteMatrix(inversed);
}

static void testTranslation()
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

static void testRotation()
{
    float h = 0.1f;
    FVector* F = new FVector[3]{ FVector(2.f, 0.f, 0.f), FVector(0.f, 3.f, 0.f), FVector(1.f, 1.f, 1.f) };
    FVector* A = new FVector[3]{ FVector(1.f, 2.f, 0.f), FVector(-1.f, -2.f, 1.f), FVector(0.f, 1.f, -1.f) };
    FVector G = FVector(0.f, 0.f, 0.f);
    Matrix I = MathLib::createMatrix(3, 3);
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

    DoubleVector result = MathLib::rotation(h, F, A, G, I, teta, tetap);
    result.print();

    delete[] F;
    delete[] A;
}