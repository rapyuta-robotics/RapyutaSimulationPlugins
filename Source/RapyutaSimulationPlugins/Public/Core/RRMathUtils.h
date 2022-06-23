/**
 * @file RRMathUtils.h
 * @brief Math utils.
 * @todo add documentation, is this necessary?
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

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
class RAPYUTASIMULATIONPLUGINS_API URRMathUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    template<typename T>
    FORCEINLINE static void BitFlagsToStack(uint32 InBitFlags, TArray<T>& OutStack)
    {
        OutStack.Reset();
        while (InBitFlags != 0)
        {
            OutStack.Add((T)FBitSet::GetAndClearNextBit(InBitFlags));
        }
    }

    // RANDOM GENERATOR --
    //
    static void InitializeRandomStream();

    template<typename T>
    FORCEINLINE static T GetRandomElement(const TArray<T>& InArray)
    {
        return (InArray.Num() > 0) ? InArray[GetRandomIntegerInRange(0, InArray.Num() - 1)] : T();
    }

    // Return an almost uniformly distributed float random number in [0, 1]
    FORCEINLINE static float GetRandomFloat()
    {
        return RandomStream.GetFraction();
    }

    FORCEINLINE static bool GetRandomBool()
    {
        return (GetRandomFloat() >= 0.5f);
    }

    // Return an almost uniformly distributed float random number between 2 float values [Min, Max]
    FORCEINLINE static float GetRandomFloatInRange(float InValueA, float InValueB)
    {
        return RandomStream.FRandRange(InValueA, InValueB);
    }

    FORCEINLINE static float GetRandomFloatInRange(const FVector2D& InValueRange)
    {
        return RandomStream.FRandRange(InValueRange.X, InValueRange.Y);
    }

    FORCEINLINE static int32 GetRandomIntegerInRange(int32 InValueA, int32 InValueB)
    {
        return RandomStream.RandRange(InValueA, InValueB);
    }

    FORCEINLINE static int32 GetRandomIntegerInRange(const FIntPoint& InValueRange)
    {
        return RandomStream.RandRange(InValueRange.X, InValueRange.Y);
    }

    FORCEINLINE static FVector GetRandomLocation(const FVector& InLocationA, const FVector& InLocationB)
    {
        return FVector(GetRandomFloatInRange(InLocationA.X, InLocationB.X),
                       GetRandomFloatInRange(InLocationA.Y, InLocationB.Y),
                       GetRandomFloatInRange(InLocationA.Z, InLocationB.Z));
    }

    FORCEINLINE static FQuat GetRandomOrientation()
    {
        // Get uniformly distributed random quaternion
        // http://planning.cs.uiuc.edu/node151.html#12337
        // http://planning.cs.uiuc.edu/node198.html
        // http://kieranwynn.github.io/pyquaternion/#from-elements
        // https://github.com/KieranWynn/pyquaternion/blob/master/pyquaternion/quaternion.py#L261
        static constexpr float C2PI = 2.f * PI;

        const float u1 = GetRandomFloat();
        const float u2 = GetRandomFloatInRange(0.f, C2PI);
        const float u3 = GetRandomFloatInRange(0.f, C2PI);
        const float sqrt_u1 = FMath::Sqrt(u1);
        const float sqrt_1_u1 = FMath::Sqrt(1 - u1);

        const float w = sqrt_1_u1 * FMath::Sin(u2);
        const float x = sqrt_1_u1 * FMath::Cos(u2);
        const float y = sqrt_u1 * FMath::Sin(u3);
        const float z = sqrt_u1 * FMath::Cos(u3);

        return FQuat(x, y, z, w);
    }

    static FVector GetRandomSphericalPosition(const FVector& InCenter,
                                              const FVector2D& InDistanceRange,
                                              const FVector2D& InHeightRange);

private:
    static FRandomStream RandomStream;
};
