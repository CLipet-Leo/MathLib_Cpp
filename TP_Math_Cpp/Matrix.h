#pragma once

#include <string>


/**
 * Class to represent a matrix with float values
 */
class Matrix
{
public:
    Matrix(int rows, int cols);
    Matrix(const Matrix& other);
    virtual ~Matrix();

    // Conversion operators
    Matrix& operator=(const Matrix& other) = default;
    Matrix& operator+=(const Matrix& other);
    float* operator[](int row);
    const float* operator[](int row) const;
    Matrix operator*(const Matrix& other) const;
    Matrix operator*(float value) const;
    Matrix operator+(const Matrix& other) const;

    // Getters
    int getRows() const { return rows; }
    int getCols() const { return cols; }
    float** getData() const { return data; }
    std::string ToString() const;

    // Static methods
    static Matrix subMatrix(const Matrix& m, int row, int col);
    static float deter(const Matrix& m);
    static Matrix com(const Matrix& m);
    static Matrix tran(const Matrix& m);
    static Matrix inverse(const Matrix& m);

private:
    int rows;
    int cols;
    float** data;
};
