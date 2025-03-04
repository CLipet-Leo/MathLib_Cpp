#pragma once
#include "json.hpp"

using json = nlohmann::json;
class FVector3;
class Matrix;
struct DoubleVector3;

namespace JsonConverter
{
	json FVector3ToJson(const FVector3& v);
	json MatrixToJson(const Matrix& m);
	json DoubleVector3ToJson(const DoubleVector3& v);
};

