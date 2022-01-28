// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/DifferentialDriveComponent.h"

DEFINE_LOG_CATEGORY(LogDifferentialDriveComponent);

void UDifferentialDriveComponent::SetWheels(UPhysicsConstraintComponent* InWheelLeft, UPhysicsConstraintComponent* InWheelRight)
{
    auto fSetWheel = [this](UPhysicsConstraintComponent*& CurWheel, UPhysicsConstraintComponent* NewWheel)
    {
        if (IsValid(NewWheel))
        {
            CurWheel = NewWheel;
            CurWheel->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
            CurWheel->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
            CurWheel->SetAngularVelocityDriveTwistAndSwing(true, false);
        }
        else
        {
            UE_LOG(LogDifferentialDriveComponent, Error, TEXT("[%s] SetWheels() NewWheel is invalid!"), *GetName());
        }
    };

    fSetWheel(WheelLeft, InWheelLeft);
    fSetWheel(WheelRight, InWheelRight);
}

void UDifferentialDriveComponent::SetPerimeter()
{
    if (WheelRadius <= 1e-6)
    {
        WheelRadius = 1.0f;
        UE_LOG(LogDifferentialDriveComponent,
               Warning,
               TEXT("[%s] Wheel radius is too small. Wheel radius is reset to 1.0"),
               *GetName());
    }
    WheelPerimeter = WheelRadius * 2.f * M_PI;
}

void UDifferentialDriveComponent::UpdateMovement(float DeltaTime)
{
    if (IsValid(WheelLeft) && IsValid(WheelRight))
    {
        float revolutionVelLeft = GetDesiredWheelSpeed(true) / WheelPerimeter;
        float revolutionVelRight = GetDesiredWheelSpeed(false) / WheelPerimeter;

        WheelLeft->SetAngularVelocityTarget(FVector(-revolutionVelLeft, 0.f, 0.f));
        WheelRight->SetAngularVelocityTarget(FVector(-revolutionVelRight, 0.f, 0.f));
        WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
        WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
    }
    else
    {
        UE_LOG(LogDifferentialDriveComponent, Error, TEXT("[%s] Wheel Joints are not set"), *GetName());
    }
}

void UDifferentialDriveComponent::UpdateOdom(float DeltaTime)
{
    if (!bIsOdomInitialized)
    {
        InitOdom();
        PoseEncoderX = 0;
        PoseEncoderY = 0;
        PoseEncoderTheta = 0;
    }

    // time
    const float timeNow = UGameplayStatics::GetTimeSeconds(this);
    OdomData.header_stamp_sec = static_cast<int32>(timeNow);
    uint64 ns = (uint64)(timeNow * 1e+09f);
    OdomData.header_stamp_nanosec = static_cast<uint32>(ns - (OdomData.header_stamp_sec * 1e+09));

    // vl and vr as computed here is ok for kinematics (as fetched from drive comp by default)
    // for physics, vl and vr should be computed based on the change in wheel orientation (i.e. the velocity term to be used is
    // wheel rotations per unit time [rad/s]) together with the wheel radius or perimeter, the displacement can be computed:
    //  vl = (left_wheel_orientation_rad_now - left_wheel_orientation_rad_previous) * perimeter / (2pi)
    //  vr = (right_wheel_orientation_rad_now - right_wheel_orientation_rad_previous) * perimeter / (2pi)
    // in the kinematics case, (dx,dy,dtheta) can be simplified considerably
    // but as this is not a performance bottleneck, for the moment we leave the full general formulation,
    // at least until the odom for the physics version of the agent is implemented, so that we have a reference

    // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L362
    // Noise added as a component of vl, vr
    // Gazebo links this Book here: Sigwart 2011 Autonomous Mobile Robots page:337
    //  seems to be Introduction to Autonomous Mobile Robots (Sigwart, Nourbakhsh, Scaramuzza)
    const double vl = GetLeftWheelSpeed() + bWithNoise * GaussianRNGPosition(Gen);
    const double vr = GetRightWheelSpeed() + bWithNoise * GaussianRNGPosition(Gen);

    const double sl = vl * DeltaTime;
    const double sr = vr * DeltaTime;
    const double ssum = sl + sr;
    const double sdiff = sl - sr;

    const double dtheta = sdiff / (2.f * WheelSeparationHalf);
    float s, c;
    FMath::SinCos(&s, &c, PoseEncoderTheta - 0.5f * dtheta);
    const double dx = ssum * 0.5f * c;
    const double dy = ssum * 0.5f * s;

    PoseEncoderX += dx;
    PoseEncoderY += dy;
    PoseEncoderTheta += dtheta;

    OdomData.pose_pose_position_x = PoseEncoderX;
    OdomData.pose_pose_position_y = PoseEncoderY;
    OdomData.pose_pose_position_z = 0;

    OdomData.pose_pose_orientation = FQuat(FVector::ZAxisVector, PoseEncoderTheta);

    const double w = dtheta / DeltaTime;
    const double v = sqrt(dx * dx + dy * dy) / DeltaTime;
    OdomData.twist_twist_angular.Z = w;
    OdomData.twist_twist_linear.X = v;
    OdomData.twist_twist_linear.Y = 0;
    OdomData.twist_twist_linear.Z = 0;

    OdomData.pose_covariance.Init(0, 36);
    OdomData.pose_covariance[0] = 0.01;
    OdomData.pose_covariance[7] = 0.01;
    OdomData.pose_covariance[14] = 1e+12;
    OdomData.pose_covariance[21] = 1e+12;
    OdomData.pose_covariance[28] = 1e+12;
    OdomData.pose_covariance[35] = 0.01;
    OdomData.twist_covariance.Init(0, 36);
    OdomData.twist_covariance[0] = 0.01;
    OdomData.twist_covariance[7] = 0.01;
    OdomData.twist_covariance[14] = 1e+12;
    OdomData.twist_covariance[21] = 1e+12;
    OdomData.twist_covariance[28] = 1e+12;
    OdomData.twist_covariance[35] = 0.01;

    // UE_LOG(LogTemp, Warning, TEXT("Input:"));
    // UE_LOG(LogTemp, Warning, TEXT("\tVel: %s, %s"), *Velocity.ToString(), *AngularVelocity.ToString());
    // UE_LOG(LogTemp, Warning, TEXT("Odometry:"));
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom Positon:\t\t\t\t%f %f from %f %f (%f)"), PoseEncoderX, PoseEncoderY, dx, dy,
    // Velocity.X); UE_LOG(LogTemp, Warning, TEXT("\tOdom Orientation:\t\t\t%s (%f)"), *OdomData.pose_pose_orientation.ToString(),
    // PoseEncoderTheta); UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistLin:\t\t\t\t%s - %f"), *OdomData.twist_twist_linear.ToString(),
    // OdomData.twist_twist_linear.Size()); UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistAng:\t\t\t\t%s"),
    // *OdomData.twist_twist_angular.ToString());
}

void UDifferentialDriveComponent::Initialize()
{
    Super::Initialize();
    SetPerimeter();
}
