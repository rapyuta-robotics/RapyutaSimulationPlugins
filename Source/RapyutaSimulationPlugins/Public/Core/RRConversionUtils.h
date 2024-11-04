/**
 * @file RRConversionUtils.h
 *
 * @brief Convertions between ROS and UE, i.e.
 * - m <-> cm
 * - UE(Left-handed) <-> ROS(Right-handed)
 *
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// rclUE
#include "Msgs/ROS2HitResult.h"
#include "Msgs/ROS2Odom.h"
#include "Msgs/ROS2Pose.h"
#include "Msgs/ROS2Time.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRTypeUtils.h"

#include "RRConversionUtils.generated.h"

UCLASS()
class URRConversionUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector ConvertHandedness(const FVector& InLocation)
    {
        return FVector(InLocation.X, -InLocation.Y, InLocation.Z);
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector2D ConvertHandedness2D(const FVector2D& InLocation)
    {
        return FVector2D(InLocation.X, -InLocation.Y);
    }

    // UE: cm, ROS: m
    template<typename T>
    static T DistanceROSToUE(const T& InROSDistance)
    {
        return 100.f * InROSDistance;
    }

    template<typename T>
    static T SizeROSToUE(const T& InROSSize)
    {
        return 100.f * InROSSize;
    }

    template<typename T>
    static T DistanceUEToROS(const T& InUESize)
    {
        return 0.01f * InUESize;
    }

    template<typename T>
    static T SizeUEToROS(const T& InUESize)
    {
        return 0.01f * InUESize;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector VectorUEToROS(const FVector& Input)
    {
        return 0.01f * ConvertHandedness(Input);
    }

    FORCEINLINE static void VectorUEToROS(const double& InputX,
                                          const double& InputY,
                                          const double& InputZ,
                                          double& OutputX,
                                          double& OutputY,
                                          double& OutputZ)
    {
        OutputX = InputX * 0.01f;
        OutputY = -InputY * 0.01f;
        OutputZ = InputZ * 0.01f;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector2D Vector2DUEToROS(const FVector2D& Input)
    {
        return 0.01f * ConvertHandedness2D(Input);
    }

    FORCEINLINE static void Vector2DUEToROS(const double& InputX, const double& InputY, double& OutputX, double& OutputY)
    {
        OutputX = InputX * 0.01f;
        OutputY = -InputY * 0.01f;
    }

    /**
     * @brief Convert UE Rotation to ROS Vector with optional Deg->Rad conversion
     * @param Input
     * @param bDegToRad
     * @return FVector
     */
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector RotationUEVectorToROS(const FVector& Input, const bool bDegToRad = true)
    {
        FVector output = Input;
        output.Y = -output.Y;
        output.Z = -output.Z;

        return bDegToRad ? FMath::DegreesToRadians(output) : output;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FRotator RotatorUEToROS(const FRotator& Input, const bool bDegToRad = true)
    {
        FRotator output = Input;
        output.Roll = bDegToRad ? FMath::DegreesToRadians(output.Roll) : output.Roll;
        output.Pitch = -bDegToRad ? FMath::DegreesToRadians(output.Pitch) : output.Pitch;
        output.Yaw = -bDegToRad ? FMath::DegreesToRadians(output.Yaw) : output.Yaw;

        return output;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FQuat QuatUEToROS(const FQuat& Input)
    {
        FQuat Output = Input;

        Output.X = -Output.X;
        Output.Z = -Output.Z;

        return Output;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FTransform TransformUEToROS(const FTransform& Input)
    {
        FTransform Output = Input;

        Output.SetTranslation(VectorUEToROS(Input.GetTranslation()));
        Output.SetRotation(QuatUEToROS(Input.GetRotation()));

        return Output;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FROSOdom OdomUEToROS(const FROSOdom& Input)
    {
        FROSOdom Output = Input;

        Output.Pose.Pose.Position = VectorUEToROS(Input.Pose.Pose.Position);
        Output.Pose.Pose.Orientation = QuatUEToROS(Output.Pose.Pose.Orientation);

        Output.Twist.Twist.Linear = VectorUEToROS(Output.Twist.Twist.Linear);
        Output.Twist.Twist.Angular = RotationUEVectorToROS(Output.Twist.Twist.Angular);

        return Output;
    }

    // ROS to UE conversion
    // m -> cm
    // rad -> degree
    // Right handed -> Left handed

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector VectorROSToUE(const FVector& Input)
    {
        return 100.f * ConvertHandedness(Input);
    }

    FORCEINLINE static void VectorROSToUE(const FVector& Input, FVector& Output)
    {
        Output.Set(Input.X * 100.f, -Input.Y * 100.f, Input.Z * 100.f);
    }

    FORCEINLINE static void VectorROSToUE(const double& InputX,
                                          const double& InputY,
                                          const double& InputZ,
                                          double& OutputX,
                                          double& OutputY,
                                          double& OutputZ)
    {
        OutputX = InputX * 100.f;
        OutputY = -InputY * 100.f;
        OutputZ = InputZ * 100.f;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector2D Vector2DROSToUE(const FVector2D& Input)
    {
        return 100.f * ConvertHandedness2D(Input);
    }

    FORCEINLINE static void Vector2DROSToUE(const FVector2D& Input, FVector2D& Output)
    {
        Output.Set(Input.X * 100.f, -Input.Y * 100.f);
    }

    FORCEINLINE static void Vector2DROSToUE(const double& InputX, const double& InputY, double& OutputX, double& OutputY)
    {
        OutputX = InputX * 100.f;
        OutputY = -InputY * 100.f;
    }

    /**
     * @brief Convert ROS Rotation Euler (rad) to UE Vector with optional Rad->Deg conversion
     * @param Input
     * @param bRadToDeg convert radian to degree or not
     * @return FVector
     */
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector RotationROSToUEVector(const FVector& Input, const bool bRadToDeg = true)
    {
        FVector output = Input;
        output.Y = -output.Y;
        output.Z = -output.Z;

        return bRadToDeg ? FMath::RadiansToDegrees(output) : output;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FRotator RotatorROSToUE(const FRotator& Input, const bool bRadToDeg = true)
    {
        FRotator output = Input;
        output.Roll = bRadToDeg ? FMath::RadiansToDegrees(output.Roll) : output.Roll;
        output.Pitch = -bRadToDeg ? FMath::RadiansToDegrees(output.Pitch) : output.Pitch;
        output.Yaw = -bRadToDeg ? FMath::RadiansToDegrees(output.Yaw) : output.Yaw;

        return output;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FQuat QuatROSToUE(const FQuat& Input)
    {
        FQuat Output = Input;

        Output.X = -Output.X;
        Output.Z = -Output.Z;

        return Output;
    }

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FTransform TransformROSToUE(const FTransform& Input)
    {
        FTransform Output = Input;

        Output.SetTranslation(VectorROSToUE(Input.GetTranslation()));
        Output.SetRotation(QuatROSToUE(Input.GetRotation()));

        return Output;
    }

    /**
     * @brief Convert Odom from ROS system to UE system.
     * @note pose is cast double to float. it will be resolved in UE5 since FVector uses double as default in UE5
     * @sa https://docs.unrealengine.com/5.0/en-US/large-world-coordinates-in-unreal-engine-5/#:~:text=Engine%205%2C%20the-,FVector,-casts%20will%20continue
     * @param Input
     * @return FROSOdom
     */
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FROSOdom OdomROSToUE(const FROSOdom& Input)
    {
        FROSOdom Output = Input;

        VectorROSToUE(Input.Pose.Pose.Position, Output.Pose.Pose.Position);
        Output.Pose.Pose.Orientation = QuatROSToUE(Output.Pose.Pose.Orientation);

        Output.Twist.Twist.Linear = VectorROSToUE(Output.Twist.Twist.Linear);
        Output.Twist.Twist.Angular = RotationROSToUEVector(Output.Twist.Twist.Angular);

        return Output;
    }

    /**
     * @brief Convert ROS Pose to UE Transform
     * @sa https://docs.unrealengine.com/5.0/en-US/large-world-coordinates-in-unreal-engine-5/#:~:text=Engine%205%2C%20the-,FVector,-casts%20will%20continue
     * @param InROSPose
     * @return FTransform
     */
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FTransform PoseROSToUETransform(const FROSPose& InROSPose)
    {
        return FTransform(URRConversionUtils::QuatROSToUE(InROSPose.Orientation),
                          URRConversionUtils::VectorROSToUE(InROSPose.Position));
    }

    // time to ROS stamp
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FROSTime FloatToROSStamp(const float InTimeSec)
    {
        FROSTime stamp;
        stamp.Sec = static_cast<int32>(InTimeSec);
        stamp.Nanosec = uint32((InTimeSec - stamp.Sec) * 1e+09f);
        return stamp;
    }

    static FROSTime GetCurrentROS2Time(const UObject* InContextObject)
    {
        return FloatToROSStamp(UGameplayStatics::GetTimeSeconds(InContextObject->GetWorld()));
    }

    static float ROSStampToFloat(const FROSTime& InTimeStamp)
    {
        return InTimeStamp.Sec + InTimeStamp.Nanosec * 1e-09f;
    }

    template<typename T>
    static FString ToString(const T& InValue)
    {
        if constexpr (TIsArithmetic<T>::Value || TIsIntegral<T>::Value)
        {
            return FString::FromInt(InValue);
        }
        else if constexpr (TIsFloatingPoint<T>::Value)
        {
            return FString::SanitizeFloat(InValue);
        }
        else if constexpr (TIsCharType<T>::Value || TIsCharPointer<T>::Value)
        {
            return FString(InValue);
        }
        else if constexpr (TIsSame<T, FName>::Value || TIsSame<T, FText>::Value)
        {
            return InValue.ToString();
        }
        else if constexpr (TIsSame<T, FString>::Value)
        {
            return InValue;
        }
        else
        {
            UE_LOG_WITH_INFO(LogTemp, Error, TEXT("[%s] is not yet supported"), TNameOf<T>::GetName());
        }
        return FString();
    }

    template<typename T>
    static FString ArrayToString(const TArray<T>& InArray, const TCHAR* InDelimiter = TEXT(","))
    {
        return FString::JoinBy(InArray, InDelimiter, [](const T& InElement) { return URRConversionUtils::ToString(InElement); });
    }

    template<typename T, std::size_t N>
    static FString ArrayToString(const T (&InArray)[N], const TCHAR* InDelimiter = TEXT(","))
    {
        TArray<FString> strArray;
        for (auto i = 0; i < URRCoreUtils::GetArraySize<T, N>(InArray); ++i)
        {
            strArray.Add(URRConversionUtils::ToString(InArray[i]));
        }
        return FString::Join(strArray, InDelimiter);
    }

    // HitResult
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FROSHitResult HitResultUEToROS(const FHitResult& InHit)
    {
        FROSHitResult result;
        result.bBlockingHit = InHit.bBlockingHit;
        result.bStartPenetrating = InHit.bStartPenetrating;
        result.ComponentName = InHit.Component->GetName();
        result.ActorName = InHit.HitObjectHandle.GetName();
        result.Distance = InHit.Distance;
        result.ElementIndex = InHit.ElementIndex;
        result.FaceIndex = InHit.FaceIndex;
        result.BoneName = InHit.BoneName.ToString();
        result.Item = InHit.Item;
        result.MyBoneName = InHit.MyBoneName.ToString();
        result.MyItem = InHit.MyItem;
        result.PenetrationDepth = InHit.PenetrationDepth;
        result.PhysicsMaterialName = InHit.PhysMaterial == nullptr ? TEXT("") : InHit.PhysMaterial->GetName();
        result.Time = InHit.Time;
        result.ImpactNormal = InHit.ImpactNormal;
        result.ImpactPoint = InHit.ImpactPoint;
        result.Normal = InHit.Normal;
        result.Location = InHit.Location;
        result.TraceStart = InHit.TraceStart;
        result.TraceEnd = InHit.TraceEnd;

        return result;
    }
};
