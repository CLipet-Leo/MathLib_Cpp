#include "JsonConverter.h"

#include "FVector3.h"
#include "Matrix.h"

json JsonConverter::FVector3ToJson(const FVector3& v)
{
	json j;
	j["Vector"] = { v.getX(), v.getY(), v.getZ() };
	return j;
}

json JsonConverter::MatrixToJson(const Matrix& m)
{
	json j;
	json matrixData;
	for (int i = 0; i < m.getRows(); i++)
	{
		json row;
		for (int j = 0; j < m.getCols(); j++)
			row.push_back(m[i][j]);
		matrixData.push_back(row);
	}
	j["Matrice"] = matrixData;
	return j;
}

json JsonConverter::DoubleVector3ToJson(const DoubleVector3& v)
{
	json j;
	j["v1"] = FVector3ToJson(v.v1);
	j["v2"] = FVector3ToJson(v.v2);
	return j;
}