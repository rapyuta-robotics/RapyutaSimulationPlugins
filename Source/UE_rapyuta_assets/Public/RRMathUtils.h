// Copyright (C) Rapyuta Robotics
#pragma once

// Native
#include "math.h"

// UE
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Math/RandomStream.h"
//#include "Math/UnrealMath.h"
//#include "GenericPlatform/GenericPlatformMath.h"

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

    // Return an almost uniformly distributed float random number between 2 float values [Min, Max]
    FORCEINLINE static float FRandRange(float InValueA, float InValueB)
    {
        return (InValueA < InValueB) ? RandomStream.FRandRange(InValueA, InValueB) : RandomStream.FRandRange(InValueB, InValueA);
    }

    FORCEINLINE static FVector GetRandomLocation(const FVector& InLocationA, const FVector& InLocationB)
    {
        return FVector(FRandRange(InLocationA.X, InLocationB.X),
                       FRandRange(InLocationA.Y, InLocationB.Y),
                       FRandRange(InLocationA.Z, InLocationB.Z));
    }
};
