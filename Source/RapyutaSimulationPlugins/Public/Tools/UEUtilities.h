// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include <Msgs/ROS2OdometryMsg.h>

#include "UEUtilities.generated.h"

namespace ConversionUtils
{
template<typename T>
inline static T DistanceUEToROS(const T& InUEDistance)
{
    return 0.01f * InUEDistance;
}

// UE to ROS conversion
// cm -> m
// Left handed -> Right handed
inline static FVector VectorUEToROS(const FVector& Input)
{
    FVector Output = Input;

    Output.X = Output.X / 100.0f;
    Output.Y = -Output.Y / 100.0f;
    Output.Z = Output.Z / 100.0f;

    return Output;
}

inline static void VectorUEToROS(const double& InputX,
                                 const double& InputY,
                                 const double& InputZ,
                                 double& OutputX,
                                 double& OutputY,
                                 double& OutputZ)
{
    OutputX = InputX / 100.0f;
    OutputY = -InputY / 100.0f;
    OutputZ = InputZ / 100.0f;
}

inline static FVector RotationUEToROS(const FVector& Input)
{
    FVector Output = Input;

    Output.Y = -Output.Y;
    Output.Z = -Output.Z;

    return Output;
}

inline static FQuat QuatUEToROS(const FQuat& Input)
{
    FQuat Output = Input;

    Output.X = -Output.X;
    Output.Z = -Output.Z;

    return Output;
}

inline static FTransform TransformUEToROS(const FTransform& Input)
{
    FTransform Output = Input;

    Output.SetTranslation(VectorUEToROS(Input.GetTranslation()));
    Output.SetRotation(QuatUEToROS(Input.GetRotation()));

    return Output;
}

inline static FROSOdometry OdomUEToROS(const FROSOdometry& Input)
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

// // ROS to UE conversion
// // m -> cm
// // Right handed -> Left handed
inline static FVector VectorROSToUE(const FVector& Input)
{
    FVector Output = Input;

    Output.X = Output.X * 100.0f;
    Output.Y = -Output.Y * 100.0f;
    Output.Z = Output.Z * 100.0f;

    return Output;
}

inline static void VectorROSToUE(const double& InputX,
                                 const double& InputY,
                                 const double& InputZ,
                                 double& OutputX,
                                 double& OutputY,
                                 double& OutputZ)
{
    OutputX = InputX * 100.0f;
    OutputY = -InputY * 100.0f;
    OutputZ = InputZ * 100.0f;
}

inline static FVector RotationROSToUE(const FVector& Input)
{
    FVector Output = Input;

    Output.Y = -Output.Y;
    Output.Z = -Output.Z;

    return Output;
}

inline static FQuat QuatROSToUE(const FQuat& Input)
{
    FQuat Output = Input;

    Output.X = -Output.X;
    Output.Z = -Output.Z;

    return Output;
}

inline static FTransform TransformROSToUE(const FTransform& Input)
{
    FTransform Output = Input;

    Output.SetTranslation(VectorROSToUE(Input.GetTranslation()));
    Output.SetRotation(QuatROSToUE(Input.GetRotation()));

    return Output;
}

inline static FROSOdometry OdomROSToUE(const FROSOdometry& Input)
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

}    // namespace ConversionUtils

UCLASS()
class UConversionUtils : public UBlueprintFunctionLibrary
{
    GENERATED_UCLASS_BODY()

    // UE to ROS conversion
    // cm -> m
    // Left handed -> Right handed

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector VectorUEToROS(const FVector& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector RotationUEToROS(const FVector& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FQuat QuatUEToROS(const FQuat& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FTransform TransformUEToROS(const FTransform& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FROSOdometry OdomUEToROS(const FROSOdometry& Input);

    // ROS to UE conversion
    // m -> cm
    // Right handed -> Left handed

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector VectorROSToUE(const FVector& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FVector RotationROSToUE(const FVector& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FQuat QuatROSToUE(const FQuat& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FTransform TransformROSToUE(const FTransform& Input);

    UFUNCTION(BlueprintCallable, Category = "Conversion")
    static FROSOdometry OdomROSToUE(const FROSOdometry& Input);
};
