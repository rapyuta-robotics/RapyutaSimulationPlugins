// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#pragma once

// Native
#include "math.h"

// UE
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Math/RandomStream.h"
//#include "Math/UnrealMath.h"
//#include "GenericPlatform/GenericPlatformMath.h"

#include "RRMathUtils.generated.h"

// https://github.com/ros2/rcpputils/blob/master/include/rcppmath/rolling_mean_accumulator.hpp
template<typename T>
class FRRRollingMeanAccumulator
{
public:
    /**
     * Constructs the rolling mean accumulator with a specified window size.
     *
     * \param[in] InRollingWindowSize The unsigned integral length of the accumulator's window length.
     */
    explicit FRRRollingMeanAccumulator(size_t InRollingWindowSize) : NextInsert(0), Sum(0.0), bBufferFilled(false)
    {
        Buffer.SetNumZeroed(InRollingWindowSize);
    }

    /**
     * Collects the provided value in the accumulator's buffer.
     *
     * \param[in] val The value to accumulate.
     */
    void Accumulate(const T& InVal)
    {
        Sum -= Buffer[NextInsert];
        Sum += InVal;
        Buffer[NextInsert] = InVal;
        NextInsert++;
        bBufferFilled |= NextInsert >= Buffer.Num();
        NextInsert = NextInsert % Buffer.Num();
    }

    /**
     * Calculates the rolling mean accumulated insofar.
     *
     * \return Rolling mean of the accumulated values.
     */
    T GetRollingMean() const
    {
        size_t validDataCount = bBufferFilled * Buffer.Num() + !bBufferFilled * NextInsert;
        check(validDataCount > 0);
        return Sum / validDataCount;
    }

private:
    TArray<T> Buffer;
    size_t NextInsert = 0;
    T Sum;
    bool bBufferFilled = false;
};

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRMathUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    FORCEINLINE static FVector ConvertHandedness(const FVector& InLocation)
    {
        return FVector(InLocation.X, -InLocation.Y, InLocation.Z);
    }

    template<typename T>
    FORCEINLINE static void BitFlagsToStack(uint32 InBitFlags, TArray<T>& OutStack)
    {
        OutStack.Reset();
        while (InBitFlags != 0)
        {
            OutStack.Add((T)FBitSet::GetAndClearNextBit(InBitFlags));
        }
    }

    static FRandomStream RandomStream;

    // Return an almost uniformly distributed float random number in [0, 1]
    FORCEINLINE static float FRand()
    {
        return RandomStream.GetFraction();
    }

    FORCEINLINE static bool RandBool()
    {
        return (FRand() >= 0.5f);
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

    static FMatrix ComputeViewMatrixFromObjectToTarget(const FTransform& InObjectTransform, const FVector& InTargetLocation);
};
