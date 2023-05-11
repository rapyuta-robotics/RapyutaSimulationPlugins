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
#include "Msgs/ROS2Odom.h"
#include "Msgs/ROS2Pose.h"
#include "Msgs/ROS2Time.h"

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
    static FVector RotationUEToROS(const FVector& Input)
    {
        FVector Output = Input;

        Output.Y = -Output.Y;
        Output.Z = -Output.Z;

        return Output;
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
        Output.Twist.Twist.Angular = RotationUEToROS(Output.Twist.Twist.Angular);

        return Output;
    }

    // ROS to UE conversion
    // m -> cm
    // Right handed -> Left handed

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector VectorROSToUE(const FVector& Input)
    {
        FVector Output = Input;

        Output.X = Output.X;
        Output.Y = -Output.Y;
        Output.Z = Output.Z;

        return 100.f * Output;
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

    /**
     * @brief Convert ROS Rotation Euler (rad) to UE Vector retaining unit as [rad]
     * @param Input
     * @return FVector
     */
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector RotationROSToUEVector(const FVector& Input)
    {
        FVector output = Input;
        output.Y = -output.Y;
        output.Z = -output.Z;
        return output;
    }

    /**
     * @brief Convert ROS Rotation (rad) to UE Rotator (deg)
     * @param InROSPose
     * @return FRotator
     */
    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FRotator RotationROSToUE(const FVector& Input)
    {
        return FRotator::MakeFromEuler(FMath::RadiansToDegrees(RotationROSToUEVector(Input)));
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
};
