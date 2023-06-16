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
    /**
     * @brief Convert uint32 BitFlags to TArray
     *
     * @tparam T
     * @param InBitFlags
     * @param OutStack
     * @return
     */
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
    // These are mostly for Velocity handling, which is inherently 3D FVector
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
        static constexpr float OVER_MAG_PERCENT = 1.01f;
        const float maxExtraMagnitude = OVER_MAG_PERCENT * FMath::Square(InMaxMagnitude);
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
     * @return True if #InVector exceeds #InMaxMagnitude
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

    /**
     * @brief Clamp a given rotator to its max axis angles
     *
     * @param InRotator
     * @param InMaxAngles > 0
     * @return Clamped rotator
     */
    FORCEINLINE static void ClampRotatorToMaxAngles(FRotator& InRotator, const FRotator& InMaxAngles)
    {
        InRotator = FRotator(ClampAngle(InRotator.Pitch, InMaxAngles.Pitch),
                             ClampAngle(InRotator.Yaw, InMaxAngles.Yaw),
                             ClampAngle(InRotator.Roll, InMaxAngles.Roll));
    }

    /**
     * @brief Clamp an angle in a range [-InMaxAxisAngle, InMaxAxisAngle] with maximum values between (-360,360)
     * @tparam T
     * @param InAngle
     * @param InMaxAngle > 0
     * @return Clamped angle
     */
    template<typename T>
    FORCEINLINE static T ClampAngle(T InAngle, const T InMaxAngle)
    {
        // returns Angle in the range (-360,360)
        InAngle = FMath::Fmod(InAngle, 360.f);
        // Both FRotator::ClampAxis() and NormalizeAxis() could possible change the sign of InAngle so not used here
        return FMath::Clamp(InAngle, -InMaxAngle, InMaxAngle);
    }

    // RANDOM GENERATOR --
    /**
     * @brief Initialize #URRMathUtils::RandomStream by FRandomStream
     * @sa [FRandomStream](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FRandomStream/)
     */
    static void InitializeRandomStream();

    /**
     * @brief Get the Random Element of given array
     *
     * @tparam T
     * @param InArray
     * @return T
     */
    template<typename T>
    FORCEINLINE static T GetRandomElement(const TArray<T>& InArray)
    {
        return (InArray.Num() > 0) ? InArray[GetRandomIntegerInRange(0, InArray.Num() - 1)] : T();
    }

    //! Return an almost uniformly distributed float random number in [0, 1]
    /**
     * @brief Return an almost uniformly distributed float random number in [0, 1]. Calls GetFraction from #URRMathUtils::RandomStream.
     * @sa [GetFraction](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FRandomStream/GetFraction/)
     * @return float
     */
    FORCEINLINE static float GetRandomBias()
    {
        return RandomStream.GetFraction();
    }

    /**
     * @brief Check value get from #GetRandomBias in [0, InBias]
     *
     * @param InBias
     * @return bool
     */
    FORCEINLINE static bool IsBiased(float InBias)
    {
        const float bias = GetRandomBias();
        return (bias > 0.f) && (bias <= InBias);
    }

    /**
     * @brief Check value get from #GetRandomBias in [InMinBias, InMaxBias]
     *
     * @param InMinBias
     * @param InMaxBias
     * @return bool
     */
    FORCEINLINE static bool IsBiased(float InMinBias, float InMaxBias)
    {
        return FMath::IsWithin(GetRandomBias(), InMinBias, InMaxBias);
    }

    /**
     * @brief Check value get from #GetRandomBias > 0.5
     *
     * @return bool
     */
    FORCEINLINE static bool GetRandomBool()
    {
        return (GetRandomBias() >= 0.5f);
    }

    /**
     * @brief Return an almost uniformly distributed float random number between 2 float values [Min, Max] by using
     * @sa [FRandRange](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FRandomStream/FRandRange/)
     * @param InValueA
     * @param InValueB
     * @return float
     */
    FORCEINLINE static float GetRandomFloatInRange(float InValueA, float InValueB)
    {
        return RandomStream.FRandRange(InValueA, InValueB);
    }

    /**
     * @brief Return an almost uniformly distributed float random number between FVector
     * @sa [FRandRange](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FRandomStream/FRandRange/)
     * @param InValueRange
     * @return float
     */
    FORCEINLINE static float GetRandomFloatInRange(const FVector2f& InValueRange)
    {
        return RandomStream.FRandRange(InValueRange.X, InValueRange.Y);
    }

    /**
     *@brief Return an almost uniformly distributed int random number between FVector
     * @sa [FRandRange](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FRandomStream/FRandRange/)
     * @param InValueA
     * @param InValueB
     * @return int32
     */
    FORCEINLINE static int32 GetRandomIntegerInRange(int32 InValueA, int32 InValueB)
    {
        return RandomStream.RandRange(InValueA, InValueB);
    }

    /**
     * @brief Return an almost uniformly distributed Int random number between 2 int values [0, Max] by using
     * @sa [FRandRange](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FRandomStream/FRandRange/)
     * @param InValueMax
     * @return int32
     */
    FORCEINLINE static int32 GetRandomIntegerInRange(int32 InValueMax)
    {
        return RandomStream.RandRange(0, InValueMax);
    }

    /**
    * @brief Return an almost uniformly distributed int random number between FIntPoint
    * @sa [FRandRange](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FRandomStream/FRandRange/)
    * @param InValueRange
    * @return int32
    */
    FORCEINLINE static int32 GetRandomIntegerInRange(const FIntPoint& InValueRange)
    {
        return RandomStream.RandRange(InValueRange.X, InValueRange.Y);
    }

    /**
     * @brief Get the Random Location between two vectors.
     *
     * @param InLocationA
     * @param InLocationB
     * @return FVector
     */
    FORCEINLINE static FVector GetRandomLocation(const FVector& InLocationA, const FVector& InLocationB)
    {
        return FVector(GetRandomFloatInRange(InLocationA.X, InLocationB.X),
                       GetRandomFloatInRange(InLocationA.Y, InLocationB.Y),
                       GetRandomFloatInRange(InLocationA.Z, InLocationB.Z));
    }

    /**
     * @brief Get uniformly distributed random quaternion
     * @sa http://planning.cs.uiuc.edu/node151.html#12337
     * @sa http://planning.cs.uiuc.edu/node198.html
     * @sa http://kieranwynn.github.io/pyquaternion/#from-elements
     * @sa https://github.com/KieranWynn/pyquaternion/blob/master/pyquaternion/quaternion.py#L261
     * @return FQuat
     */
    FORCEINLINE static FQuat GetRandomOrientation()
    {
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

    /**
     * @brief Return a random number between [0, 360]

     * @return float
     */
    FORCEINLINE static float GetRandomYawInDegrees()
    {
        return GetRandomFloatInRange(0.f, 360.f);
    }

    /**
     * @brief  Return a random number between [-InMaxExtent, InMaxExtent]

     *
     * @param InMaxExtent
     * @return float
     */
    FORCEINLINE static float GetRandomExtent(float InMaxExtent)
    {
        return GetRandomFloatInRange(-InMaxExtent, InMaxExtent);
    }

    /**
     * @brief Get the Random Spherical Position object
     *
     * @param InCenter
     * @param InDistanceRange
     * @param InHeightRange
     * @return FVector
     */
    static FVector GetRandomSphericalPosition(const FVector& InCenter,
                                              const FVector2f& InDistanceRange,
                                              const FVector2f& InHeightRange);

    /**
     * @brief Get the Random Color From H S V object
     *
     * @return FLinearColor
     */
    FORCEINLINE static FLinearColor GetRandomColorFromHSV(const float InMin = 0.6f)
    {
        FLinearColor color(GetRandomYawInDegrees(),               // Hue
                           GetRandomBias(),                       // Saturation
                           GetRandomFloatInRange(InMin, 1.f));    // Value

        return color.HSVToLinearRGB();
    }

    /**
     * @brief Get the Random Color From H S V object
     *
     * @param InHSVRange
     * @return FLinearColor
     */
    FORCEINLINE static FLinearColor GetRandomColorFromHSV(const TArray<FVector2D>& InHSVRange)
    {
        FLinearColor color(GetRandomFloatInRange(InHSVRange[0].X, InHSVRange[0].Y),     // Hue
                           GetRandomFloatInRange(InHSVRange[1].X, InHSVRange[1].Y),     // Saturation
                           GetRandomFloatInRange(InHSVRange[2].X, InHSVRange[2].Y));    // Value

        return color.HSVToLinearRGB();
    }

    /**
     * @brief Get the Random Color
     *
     * @return FLinearColor
     */
    FORCEINLINE static FLinearColor GetRandomColor()
    {
        return FLinearColor(GetRandomBias(), GetRandomBias(), GetRandomBias(), GetRandomBias());
    }

    /**
     * @brief update current value with step to reach target within tolerance
     * 
     * @param current 
     * @param target 
     * @param step 
     * @param tolerance 
     */
    FORCEINLINE static bool StepUpdate(double& current, const double target, const double step, const double tolerance)
    {
        bool reached = false;
        double diff = target - current;
        double absStep = FMath::Abs(step);
        double signedStep = diff > 0 ? absStep : -absStep;
        if(FMath::Abs(diff)<=absStep || //can reach target in step
            FMath::Abs(diff)<=FMath::Abs(tolerance) || //with in tolerance
            (signedStep > 0 && diff < 0) || (signedStep < 0 && diff > 0) //overshoot
        )
        {
            current = target;
            reached = true;
        }
        else 
        {
            current += signedStep;
        }

        return reached;
    }

    /**
     * @brief update current value with step to reach target within tolerance
     * 
     * @param current 
     * @param target 
     * @param step 
     * @param tolerance 
     */
    FORCEINLINE static bool StepUpdateAngle(double& current, const double target, const double step, const double tolerance)
    {
        bool reached = false;
        double currentNormalized = FRotator::NormalizeAxis(current);
        const double targetNormalized = FRotator::NormalizeAxis(target);
        double diff = FRotator::NormalizeAxis(targetNormalized  - currentNormalized);
        double absStep = FMath::Abs(step);
        double signedStep = diff > 0 ? absStep : -absStep;
        if(FMath::Abs(diff)<=absStep || //can reach target in step
            FMath::Abs(diff)<=FMath::Abs(tolerance) || //with in tolerance
            (signedStep > 0 && diff < 0) || (signedStep < 0 && diff > 0) //overshoot
        )
        {
            currentNormalized = targetNormalized;
            reached = true;
        }
        else
        {
            currentNormalized += signedStep;
            currentNormalized = FRotator::NormalizeAxis(currentNormalized);
        }

        current = currentNormalized;
        return reached;
    }

private:
    static FRandomStream RandomStream;
};
