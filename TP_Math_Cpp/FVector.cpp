#include "FVector.h"
#include <sstream>
#include <cmath>

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

FVector FVector::prodValue(const FVector& v, float value)
{
	return {v.X * value, v.Y * value, v.Z * value};
}

FVector FVector::divValue(const FVector& v, float value)
{
	return {v.X / value, v.Y / value, v.Z / value};
}

FVector FVector::prodVect(const FVector& u, const FVector& v)
{
	return {u.Y * v.Z - u.Z * v.Y, u.Z * v.X - u.X * v.Z, u.X * v.Y - u.Y * v.X};
}

FVector FVector::prodMatrix(const FVector& v, const Matrix& m)
{
	return {
		v.X * m.data[0][0] + v.Y * m.data[0][1] + v.Z * m.data[0][2],
		v.X * m.data[1][0] + v.Y * m.data[1][1] + v.Z * m.data[1][2],
		v.X * m.data[2][0] + v.Y * m.data[2][1] + v.Z * m.data[2][2]
	};
}

FVector FVector::moment(const FVector& F, const FVector& A, const FVector& G)
{
	return prodVect((A - G), F);
}
