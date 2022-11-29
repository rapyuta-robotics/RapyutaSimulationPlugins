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

    // VECTOR --
    /**
     * @brief Check if a vector's magnitude exceeds a given max value
     *
     * @param InVector
     * @param InMaxMagnitude
     * @param b2D Whether only X,Y are accounted
     */
    FORCEINLINE static bool IsVectorExceedingMaxMagnitude(const FVector& InVector, float InMaxMagnitude, bool b2D)
    {
        // Give 1% error tolerance, to account for numeric imprecision
        static constexpr float OVER_VEL_PERCENT = 1.01f;
        const float maxExtraMagnitude = OVER_VEL_PERCENT * FMath::Square(InMaxMagnitude);
        return (maxExtraMagnitude > 0.f) &&
               (b2D ? (InVector.SizeSquared2D() > maxExtraMagnitude) : (InVector.SizeSquared() > maxExtraMagnitude));
    }

    /**
     * @brief Set a given vector's magnitude
     *
     * @param InVector
     * @param InMaxMagnitude
     * @param b2D Whether only X,Y are accounted
     */
    FORCEINLINE static void SetVectorClampedToMaxMagnitude(FVector& InVector, float InMaxMagnitude, bool b2D)
    {
        if (b2D)
        {
            InVector = InVector.GetClampedToMaxSize2D(InMaxMagnitude);
        }
        else
        {
            InVector = InVector.GetClampedToMaxSize(InMaxMagnitude);
        }
    }

    /**
     * @brief Clamp a given vector's magnitude to its max magnitude
     *
     * @param InVector
     * @param InMaxMagnitude
     * @param b2D Whether only X,Y are accounted
     */
    FORCEINLINE static bool ClampVectorToMaxMagnitude(FVector& InVector, float InMaxMagnitude, bool b2D)
    {
        if (IsVectorExceedingMaxMagnitude(InVector, InMaxMagnitude, b2D))
        {
            SetVectorClampedToMaxMagnitude(InVector, InMaxMagnitude, b2D);
            return true;
        }
        return false;
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
    FORCEINLINE static float GetRandomBias()
    {
        return RandomStream.GetFraction();
    }

    FORCEINLINE static bool IsBiased(float InBias)
    {
        const float bias = GetRandomBias();
        return (bias > 0.f) && (bias <= InBias);
    }

    FORCEINLINE static bool IsBiased(float InMinBias, float InMaxBias)
    {
        return FMath::IsWithin(GetRandomBias(), InMinBias, InMaxBias);
    }

    FORCEINLINE static bool GetRandomBool()
    {
        return (GetRandomBias() >= 0.5f);
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
    FORCEINLINE static int32 GetRandomIntegerInRange(int32 InValueMax)
    {
        return RandomStream.RandRange(0, InValueMax);
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

        const float u1 = GetRandomBias();
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

    FORCEINLINE static float GetRandomYawInDegrees()
    {
        return GetRandomFloatInRange(0.f, 360.f);
    }

    FORCEINLINE static float GetRandomExtent(float InMaxExtent)
    {
        return GetRandomFloatInRange(-InMaxExtent, InMaxExtent);
    }

    static FVector GetRandomSphericalPosition(const FVector& InCenter,
                                              const FVector2D& InDistanceRange,
                                              const FVector2D& InHeightRange);

    FORCEINLINE static FLinearColor GetRandomColorFromHSV()
    {
        FLinearColor color(GetRandomYawInDegrees(),              // Hue
                           GetRandomBias(),                      // Saturation
                           GetRandomFloatInRange(0.6f, 1.f));    // Value

        return color.HSVToLinearRGB();
    }

    FORCEINLINE static FLinearColor GetRandomColor()
    {
        return FLinearColor(GetRandomBias(), GetRandomBias(), GetRandomBias(), GetRandomBias());
    }

private:
    static FRandomStream RandomStream;
};
