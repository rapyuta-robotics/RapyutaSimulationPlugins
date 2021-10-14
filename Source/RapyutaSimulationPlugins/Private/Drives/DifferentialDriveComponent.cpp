// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/DifferentialDriveComponent.h"

DEFINE_LOG_CATEGORY(LogDifferentialDriveComponent);

UDifferentialDriveComponent::UDifferentialDriveComponent()
{
}

void UDifferentialDriveComponent::SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight)
{
    WheelLeft = InWheelLeft;
    WheelRight = InWheelRight;

    WheelLeft->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
    WheelLeft->SetAngularVelocityDriveTwistAndSwing(true, false);

    WheelRight->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
    WheelRight->SetAngularVelocityDriveTwistAndSwing(true, false);
}

void UDifferentialDriveComponent::SetPerimeter()
{
    if (WheelRadius <= 1e-6)
    {
        UE_LOG(LogDifferentialDriveComponent, Warning, TEXT("Wheel radius is too small. Wheel radius is reset to 1.0"));
        WheelRadius = 1.0f;
    }
    WheelPerimeter = WheelRadius * 2.0 * 3.1416;
}

void UDifferentialDriveComponent::UpdateMovement(float DeltaTime)
{
    if (IsValid(WheelLeft) && IsValid(WheelRight))
    {
        float velL = Velocity.X + AngularVelocity.Z * WheelSeparationHalf;
        float velR = Velocity.X - AngularVelocity.Z * WheelSeparationHalf;

        WheelLeft->SetAngularVelocityTarget(FVector(-velL / WheelPerimeter, 0, 0));
        WheelRight->SetAngularVelocityTarget(FVector(-velR / WheelPerimeter, 0, 0));
        WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
        WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
    }
    else
    {
        UE_LOG(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }
}

void UDifferentialDriveComponent::UpdateOdom(float DeltaTime)
{
    // need to add noise!
    
    if (!IsOdomInitialized)
    {
        InitOdom();
        PoseEncoderX = 0;
        PoseEncoderY = 0;
        PoseEncoderTheta = 0;
    }

    // time
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    OdomData.header_stamp_sec = static_cast<int32>(TimeNow);
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    OdomData.header_stamp_nanosec = static_cast<uint32>(ns - (OdomData.header_stamp_sec * 1e+09));

	OdomData.header_frame_id = FString("odom");
	OdomData.child_frame_id = FString("base_footprint");

    // this part is based on gazebo's diff drive:
    // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/231a7219b36b8a6cdd100b59f66a3df2955df787/gazebo_plugins/src/gazebo_ros_diff_drive.cpp
    // Book: Sigwart 2011 Autonomous Mobile Robots page:337
    float sl = (Velocity.X + AngularVelocity.Z * WheelSeparationHalf + WithNoise * GaussianRNGPosition(Gen)) * DeltaTime;
    float sr = (Velocity.X - AngularVelocity.Z * WheelSeparationHalf + WithNoise * GaussianRNGPosition(Gen)) * DeltaTime;
    float ssum = sl + sr;

    float sdiff = sr - sl;

    float dx = ssum * .5f * cos(PoseEncoderTheta + sdiff / (4.f * WheelSeparationHalf));
    float dy = ssum * .5f * sin(PoseEncoderTheta + sdiff / (4.f * WheelSeparationHalf));
    float dtheta = -sdiff / (2.f * WheelSeparationHalf);

    PoseEncoderX += dx;
    PoseEncoderY += dy;
    PoseEncoderTheta += dtheta;

    float w = dtheta / DeltaTime;
    float v = sqrt(dx * dx + dy * dy) / DeltaTime;

    // FRotator is in degrees, while PoseEncoderTheta is in Radians
    FQuat qt(FRotator(0, FMath::RadiansToDegrees(PoseEncoderTheta), 0));

    OdomData.pose_pose_position_x = PoseEncoderX;
    OdomData.pose_pose_position_y = PoseEncoderY;
    OdomData.pose_pose_position_z = 0;

    OdomData.pose_pose_orientation.X = qt.X;
    OdomData.pose_pose_orientation.Y = qt.Y;
    OdomData.pose_pose_orientation.Z = qt.Z;
    OdomData.pose_pose_orientation.W = qt.W;

    OdomData.twist_twist_angular.Z = w;
    OdomData.twist_twist_linear.X = v;
    OdomData.twist_twist_linear.Y = 0;
    OdomData.twist_twist_linear.Z = 0;


	OdomData.pose_covariance.Init(0,36);
	OdomData.pose_covariance[0] = 0.01;
	OdomData.pose_covariance[7] = 0.01;
	OdomData.pose_covariance[14] = 1000000000000.0;
	OdomData.pose_covariance[21] = 1000000000000.0;
	OdomData.pose_covariance[28] = 1000000000000.0;
	OdomData.pose_covariance[35] = 0.01;
	OdomData.twist_covariance.Init(0,36);
	OdomData.twist_covariance[0] = 0.01;
	OdomData.twist_covariance[7] = 0.01;
	OdomData.twist_covariance[14] = 1000000000000.0;
	OdomData.twist_covariance[21] = 1000000000000.0;
	OdomData.twist_covariance[28] = 1000000000000.0;
	OdomData.twist_covariance[35] = 0.01;

    // UE_LOG(LogTemp, Warning, TEXT("Input:"));
    // UE_LOG(LogTemp, Warning, TEXT("\tVel: %s, %s"), *Velocity.ToString(), *AngularVelocity.ToString());
    // UE_LOG(LogTemp, Warning, TEXT("Odometry:"));
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom Positon:\t\t\t\t%f %f from %f %f (%f)"), PoseEncoderX, PoseEncoderY, dx, dy, Velocity.X);
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom Orientation:\t\t\t%s (%f)"), *OdomData.pose_pose_orientation.ToString(), PoseEncoderTheta);
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistLin:\t\t\t\t%s - %f"), *OdomData.twist_twist_linear.ToString(), OdomData.twist_twist_linear.Size());
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistAng:\t\t\t\t%s"), *OdomData.twist_twist_angular.ToString());
}

void UDifferentialDriveComponent::InitMovementComponent()
{
    Super::InitMovementComponent();

    if (!IsValid(WheelLeft) || !IsValid(WheelRight))
    {
        UE_LOG(LogDifferentialDriveComponent, Error, TEXT("Wheel Joints are not set"));
    }

    SetPerimeter();
}