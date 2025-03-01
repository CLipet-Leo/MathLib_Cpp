#include "Matrix.h"

#include <iostream>
#include <stdexcept>
#include <sstream>

Matrix::Matrix(int rows, int cols)
    : rows(rows), cols(cols)
{
    data = new float* [rows];
    for (int i = 0; i < rows; i++)
    {
        data[i] = new float[cols];
        for (int j = 0; j < cols; j++)
            data[i][j] = 0;
    }
}

Matrix::~Matrix()
{
    for (int i = 0; i < rows; i++)
        delete[] data[i];
    delete[] data;
}

Matrix& Matrix::operator+=(const Matrix& other)
{
    return *this = *this + other;
}

float* Matrix::operator[](int row)
{
    if (row < 0 || row >= rows)
        throw std::out_of_range("Row index out of range.");
    return data[row];
}

const float* Matrix::operator[](int row) const
{
    if (row < 0 || row >= rows)
        throw std::out_of_range("Row index out of range.");
    return data[row];
}

Matrix Matrix::operator*(const Matrix& other) const
{
    if (cols != other.getRows())
        throw std::invalid_argument("Matrix dimensions do not match for multiplication.");
    Matrix result(rows, other.getCols());
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < other.getCols(); j++)
        {
            for (int k = 0; k < cols; k++)
                result[i][j] += data[i][k] * other[k][j];
        }
    }
    return result;
}

Matrix Matrix::operator*(float value) const
{
    Matrix result(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result[i][j] = data[i][j] * value;
    return result;
}

Matrix Matrix::operator+(const Matrix& other) const
{
    if (rows != other.getRows() || cols != other.getCols())
        throw std::invalid_argument("Matrix dimensions do not match for addition.");
    Matrix result(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result[i][j] = data[i][j] + other[i][j];
    return result;
}

std::string Matrix::ToString() const
{
    std::ostringstream oss;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
            oss << getData()[i][j] << " ";
        oss << "\n";
    }
    return oss.str();
}

Matrix Matrix::subMatrix(const Matrix& m, int row, int col)
{
    Matrix newMat(m.getRows() - 1, m.getCols() - 1);
    int sub_i = 0;
    int sub_j = 0;
    for (int i = 0; i < m.getRows(); i++)
    {
        if (i == row)
            continue;
        for (int j = 0; j < m.getCols(); j++)
        {
            if (j == col)
                continue;
            newMat[sub_i][sub_j] = m[i][j];
            sub_j++;
        }
        sub_i++;
        sub_j = 0;
    }
    return newMat;
}

float Matrix::deter(const Matrix& m)
{
    float determinant = 0;
    if (m.getRows() == 1 && m.getCols() == 1)
        determinant = m[0][0];
    else
    {
        int j = 0;
        for (int i = 0; i < m.getRows(); i++)
            determinant += static_cast<float>(pow(-1, i + j) * m[i][j] * deter(subMatrix(m, i, j)));
    }
    return determinant;
}

Matrix Matrix::com(const Matrix& m)
{
    Matrix comatrix(m.getRows(), m.getCols());
    if (deter(m) == 0)
        throw std::runtime_error("Matrix determinant is zero, cannot compute comatrix.");
    for (int i = 0; i < m.getRows(); i++)
    {
        for (int j = 0; j < m.getCols(); j++)
        {
            comatrix[i][j] = static_cast<float>(pow(-1, i + j) * deter(subMatrix(m, i, j)));
        }
    }
    return comatrix;
}

Matrix Matrix::tran(const Matrix& m)
{
    Matrix transpose(m.getCols(), m.getRows());
    for (int i = 0; i < m.getRows(); i++)
        for (int j = 0; j < m.getCols(); j++)
            transpose[j][i] = m[i][j];
    return transpose;
}

Matrix Matrix::inverse(const Matrix& m)
{
    return tran(com(m)) * (1 / deter(m));
}

