#pragma once

#include <iostream>
#include <string>

class Matrix;

/**
 * A class to represent a 3D vector with float values
 */
class FVector3
{
public:
	FVector3();
	FVector3(float X, float Y, float Z);
	FVector3(const FVector3& other) = default;
	virtual ~FVector3() = default;

	// Conversion operators
	FVector3& operator=(const FVector3& other);
	FVector3& operator+=(const FVector3& other);
	FVector3 operator+(const FVector3& other) const;
	FVector3 operator-(const FVector3& other) const;
	FVector3 operator*(const FVector3& other) const;
	FVector3 operator*(const Matrix& matrix) const;
	FVector3 operator*(float value) const;
	FVector3 operator/(float value) const;

	std::string ToString() const;

	static FVector3 Zero() { return FVector3(0, 0, 0); }

	static FVector3 distance(const FVector3& u, const FVector3& v);
	static FVector3 prodVect(const FVector3& u, const FVector3& v);
	static FVector3 moment(const FVector3& F, const FVector3& A, const FVector3& G);

	// Getters
	float getX() const { return X; }
	float getY() const { return Y; }
	float getZ() const { return Z; }

private:
	float X;
	float Y;
	float Z;
};

// Struct to hold two FVector3 objects
struct DoubleVector3 {
	FVector3 v1;
	FVector3 v2;

	DoubleVector3(FVector3 v1, FVector3 v2) : v1(v1), v2(v2) {}

	void print() const {
		std::cout << "DoubleVector3" << '\n';
		std::cout << "v1: " << v1.ToString() << '\n';
		std::cout << "v2: " << v2.ToString() << '\n';
	}
};
