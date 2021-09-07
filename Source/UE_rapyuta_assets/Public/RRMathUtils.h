// Copyright (C) Rapyuta Robotics
#pragma once

// Native
#include "math.h"

// UE
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Math/RandomStream.h"

#include "RRMathUtils.generated.h"

UCLASS()
class UE_RAPYUTA_ASSETS_API URRMathUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    static FRandomStream RandomStream;

    // Return an almost uniformly distributed float random number in [0, 1]
    FORCEINLINE static float FRand()
    {
        return RandomStream.GetFraction();
    }

    // Return an almost uniformly distributed float random number in [Min, Max]
    FORCEINLINE static float FRandRange(float InMin, float InMax)
    {
        if (InMin < InMax)
        {
            return RandomStream.FRandRange(InMin, InMax);
        }
        else
        {
            return InMin;
        }
    }
};
