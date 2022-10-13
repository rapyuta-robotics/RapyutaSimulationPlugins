// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/DifferentialDriveComponent.h"

#include "rclcUtilities.h"

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
        float velL = Velocity.X + AngularVelocity.Z * WheelSeparationHalf;
        float velR = Velocity.X - AngularVelocity.Z * WheelSeparationHalf;

        WheelLeft->SetAngularVelocityTarget(FVector(-velL / WheelPerimeter, 0, 0));
        WheelRight->SetAngularVelocityTarget(FVector(-velR / WheelPerimeter, 0, 0));
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
    // need to add noise!

    if (!IsOdomInitialized)
    {
        InitOdom();
        PoseEncoderX = 0;
        PoseEncoderY = 0;
        PoseEncoderTheta = 0;
    }

    // time
    OdomData.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));
    
    // vl and vr as computed here is ok for kinematics
    // for physics, vl and vr should be computed based on the change in wheel orientation (i.e. the velocity term to be used is
    // wheel rotations per unit time [rad/s]) together with the wheel radius or perimeter, the displacement can be computed:
    //  vl = (left_wheel_orientation_rad_now - left_wheel_orientation_rad_previous) * perimeter / (2pi)
    //  vr = (right_wheel_orientation_rad_now - right_wheel_orientation_rad_previous) * perimeter / (2pi)
    // in the kinematics case, (dx,dy,dtheta) can be simplified considerably
    // but as this is not a performance bottleneck, for the moment we leave the full general formulation,
    // at least until the odom for the physics version of the agent is implemented, so that we have a reference
    float vl = Velocity.X + AngularVelocity.Z * WheelSeparationHalf;
    float vr = Velocity.X - AngularVelocity.Z * WheelSeparationHalf;

    // noise added as a component of vl, vr
    // Gazebo links this Book here: Sigwart 2011 Autonomous Mobile Robots page:337
    //  seems to be Introduction to Autonomous Mobile Robots (Sigwart, Nourbakhsh, Scaramuzza)
    float sl = (vl + WithNoise * GaussianRNGPosition(Gen)) * DeltaTime;
    float sr = (vr + WithNoise * GaussianRNGPosition(Gen)) * DeltaTime;
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

    OdomData.Pose.Pose.Position.X = PoseEncoderX;
    OdomData.Pose.Pose.Position.Y = PoseEncoderY;
    OdomData.Pose.Pose.Position.Z = 0;

    OdomData.Pose.Pose.Orientation = qt;

    OdomData.Twist.Twist.Angular.Z = w;
    OdomData.Twist.Twist.Linear.X = v;
    OdomData.Twist.Twist.Linear.Y = 0;
    OdomData.Twist.Twist.Linear.Z = 0;

    OdomData.Pose.Covariance[0] = 0.01;
    OdomData.Pose.Covariance[7] = 0.01;
    OdomData.Pose.Covariance[14] = 1e+12;
    OdomData.Pose.Covariance[21] = 1e+12;
    OdomData.Pose.Covariance[28] = 1e+12;
    OdomData.Pose.Covariance[35] = 0.01;
    OdomData.Twist.Covariance[0] = 0.01;
    OdomData.Twist.Covariance[7] = 0.01;
    OdomData.Twist.Covariance[14] = 1e+12;
    OdomData.Twist.Covariance[21] = 1e+12;
    OdomData.Twist.Covariance[28] = 1e+12;
    OdomData.Twist.Covariance[35] = 0.01;

    // UE_LOG(LogTemp, Warning, TEXT("Input:"));
    // UE_LOG(LogTemp, Warning, TEXT("\tVel: %s, %s"), *Velocity.ToString(), *AngularVelocity.ToString());
    // UE_LOG(LogTemp, Warning, TEXT("Odometry:"));
    // UE_LOG(LogTemp, Warning, TEXT("\tOdom Positon:\t\t\t\t%f %f from %f %f (%f)"), PoseEncoderX, PoseEncoderY, dx, dy,
    // Velocity.X); UE_LOG(LogTemp, Warning, TEXT("\tOdom Orientation:\t\t\t%s (%f)"), *OdomData.Pose.Pose.Orientation.ToString(),
    // PoseEncoderTheta); UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistLin:\t\t\t\t%s - %f"), *OdomData.Twist.Twist.Linear.ToString(),
    // OdomData.Twist.Twist.Linear.Size()); UE_LOG(LogTemp, Warning, TEXT("\tOdom TwistAng:\t\t\t\t%s"),
    // *OdomData.Twist.Twist.Angular.ToString());
}

void UDifferentialDriveComponent::Initialize()
{
    Super::Initialize();
    SetPerimeter();
}
