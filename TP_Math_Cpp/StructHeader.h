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

    void print() const
    {
        std::cout << "MovementResult" << '\n';
        std::cout << "newW: " << newW.ToString() << '\n';
        std::cout << "newG: " << newG.ToString() << '\n';
        std::cout << "newV: " << newV.ToString() << '\n';
        std::cout << "newTeta: " << newTeta.ToString() << '\n';
        std::cout << "newTetap: " << newTetap.ToString() << '\n';
    }
};
