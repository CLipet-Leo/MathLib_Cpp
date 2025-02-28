#include "FVector.h"
#include <sstream>
#include <cmath>
#include "Matrix.h"

FVector::FVector()
	: X(0), Y(0), Z(0)
{
}

FVector::FVector(float X, float Y, float Z)
	: X(X), Y(Y), Z(Z)
{
}

FVector FVector::operator+(const FVector& other) const
{
	return { X + other.getX(), Y + other.getY(), Z + other.getZ() };
}

FVector FVector::operator-(const FVector& other) const
{
	return { X - other.getX(), Y - other.getY(), Z - other.getZ() };
}

FVector FVector::operator*(const FVector& other) const
{
	return { X * other.getX(), Y * other.getY(), Z * other.getZ() };
}

FVector FVector::operator*(const Matrix& matrix) const
{
	return {
		X * matrix[0][0] + Y * matrix[0][1] + Z * matrix[0][2],
		X * matrix[1][0] + Y * matrix[1][1] + Z * matrix[1][2],
		X * matrix[2][0] + Y * matrix[2][1] + Z * matrix[2][2]
	};
}

FVector FVector::operator*(float value) const
{
	return { X * value, Y * value, Z * value };
}

FVector FVector::operator/(float value) const
{
	return { X / value, Y / value, Z / value };
}

std::string FVector::ToString() const
{
	std::ostringstream oss;
	oss << "X: " << X << ", Y: " << Y << ", Z: " << Z;
	return oss.str();
}

FVector FVector::distance(const FVector& u, const FVector& v)
{
	return {std::abs(u.X - v.X), std::abs(u.Y - v.Y), std::abs(u.Z - v.Z)};
}

FVector FVector::prodVect(const FVector& u, const FVector& v)
{
	return {u.Y * v.Z - u.Z * v.Y, u.Z * v.X - u.X * v.Z, u.X * v.Y - u.Y * v.X};
}

FVector FVector::moment(const FVector& F, const FVector& A, const FVector& G)
{
	return prodVect((A - G), F);
}
