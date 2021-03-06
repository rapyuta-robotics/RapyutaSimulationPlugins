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
#include <Msgs/ROS2OdometryMsg.h>

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
    static FVector VectorUEToROS(const FVector& Input)
    {
        FVector Output = Input;

        Output.X = Output.X;
        Output.Y = -Output.Y;
        Output.Z = Output.Z;

        return 0.01f * Output;
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
    static FROSOdometry OdomUEToROS(const FROSOdometry& Input)
    {
        FROSOdometry Output = Input;

        VectorUEToROS(Input.pose_pose_position_x,
                      Input.pose_pose_position_y,
                      Input.pose_pose_position_z,
                      Output.pose_pose_position_x,
                      Output.pose_pose_position_y,
                      Output.pose_pose_position_z);
        Output.pose_pose_orientation = QuatUEToROS(Output.pose_pose_orientation);

        Output.twist_twist_linear = VectorUEToROS(Output.twist_twist_linear);
        Output.twist_twist_angular = RotationUEToROS(Output.twist_twist_angular);

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
    static FVector RotationROSToUE(const FVector& Input)
    {
        FVector Output = Input;

        Output.Y = -Output.Y;
        Output.Z = -Output.Z;

        return Output;
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

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FROSOdometry OdomROSToUE(const FROSOdometry& Input)
    {
        FROSOdometry Output = Input;

        VectorROSToUE(Input.pose_pose_position_x,
                      Input.pose_pose_position_y,
                      Input.pose_pose_position_z,
                      Output.pose_pose_position_x,
                      Output.pose_pose_position_y,
                      Output.pose_pose_position_z);
        Output.pose_pose_orientation = QuatROSToUE(Output.pose_pose_orientation);

        Output.twist_twist_linear = VectorROSToUE(Output.twist_twist_linear);
        Output.twist_twist_angular = RotationROSToUE(Output.twist_twist_angular);

        return Output;
    }
};
