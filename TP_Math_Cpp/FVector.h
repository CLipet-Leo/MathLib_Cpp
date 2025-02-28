#pragma once

#include <iostream>
#include <string>

class Matrix;

class FVector
{
public:
	FVector();
	FVector(float X, float Y, float Z);
	virtual ~FVector() = default;

	// Conversion operators
	FVector operator+(const FVector& other) const;
	FVector operator-(const FVector& other) const;
	FVector operator*(const FVector& other) const;
	FVector operator*(const Matrix& matrix) const;
	FVector operator*(float value) const;
	FVector operator/(float value) const;

	std::string ToString() const;

	static FVector Zero() { return FVector(0, 0, 0); }

	static FVector distance(const FVector& u, const FVector& v);
	static FVector prodVect(const FVector& u, const FVector& v);
	static FVector moment(const FVector& F, const FVector& A, const FVector& G);

	// Getters
	float getX() const { return X; }
	float getY() const { return Y; }
	float getZ() const { return Z; }

private:
	float X;
	float Y;
	float Z;
};

struct DoubleVector {
	FVector v1;
	FVector v2;

	DoubleVector(FVector v1, FVector v2) : v1(v1), v2(v2) {}

	void print() const {
		std::cout << "DoubleVector" << '\n';
		std::cout << "v1: " << v1.ToString() << '\n';
		std::cout << "v2: " << v2.ToString() << '\n';
	}
};
