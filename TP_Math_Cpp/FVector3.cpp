#include "FVector3.h"
#include <sstream>
#include <cmath>
#include "Matrix.h"

FVector3::FVector3()
	: X(0), Y(0), Z(0)
{
}

FVector3::FVector3(float X, float Y, float Z)
	: X(X), Y(Y), Z(Z)
{
}

FVector3 FVector3::operator+(const FVector3& other) const
{
	return { X + other.getX(), Y + other.getY(), Z + other.getZ() };
}

FVector3 FVector3::operator-(const FVector3& other) const
{
	return { X - other.getX(), Y - other.getY(), Z - other.getZ() };
}

FVector3 FVector3::operator*(const FVector3& other) const
{
	return { X * other.getX(), Y * other.getY(), Z * other.getZ() };
}

FVector3 FVector3::operator*(const Matrix& matrix) const
{
	return {
		X * matrix[0][0] + Y * matrix[0][1] + Z * matrix[0][2],
		X * matrix[1][0] + Y * matrix[1][1] + Z * matrix[1][2],
		X * matrix[2][0] + Y * matrix[2][1] + Z * matrix[2][2]
	};
}

FVector3 FVector3::operator*(float value) const
{
	return { X * value, Y * value, Z * value };
}

FVector3 FVector3::operator/(float value) const
{
	return { X / value, Y / value, Z / value };
}

std::string FVector3::ToString() const
{
	std::ostringstream oss;
	oss << "X: " << X << ", Y: " << Y << ", Z: " << Z;
	return oss.str();
}

FVector3 FVector3::distance(const FVector3& u, const FVector3& v)
{
	return {std::abs(u.X - v.X), std::abs(u.Y - v.Y), std::abs(u.Z - v.Z)};
}

FVector3 FVector3::prodVect(const FVector3& u, const FVector3& v)
{
	return {u.Y * v.Z - u.Z * v.Y, u.Z * v.X - u.X * v.Z, u.X * v.Y - u.Y * v.X};
}

FVector3 FVector3::moment(const FVector3& F, const FVector3& A, const FVector3& G)
{
	return prodVect((A - G), F);
}
