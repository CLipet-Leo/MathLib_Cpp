#pragma once
#include "FVector3.h"
#include "Matrix.h"

struct MovementResult
{
    Matrix newW;
    FVector3 newG;
    FVector3 newV;
    FVector3 newTeta;
    FVector3 newTetap;
};
