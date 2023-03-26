// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRPoseOdomComponent.h"

URRPoseOdomComponent::URRPoseOdomComponent()
{
    SensorPublisherClass = URRROS2OdomPublisher::StaticClass();
    TopicName = TEXT("odom");
    PublicationFrequencyHz = 30;
    FrameId = TEXT("odom"); //default frame id
}

void URRPoseOdomComponent::BeginPlay()
{
    Super::BeginPlay();
    InitOdom();
}

void URRPoseOdomComponent::SensorUpdate()
{
    if (!ManualUpdate)
    {
        float currentTime = UGameplayStatics::GetTimeSeconds(GetWorld());
        UpdateOdom(currentTime - LastUpdatedTime);
        LastUpdatedTime = currentTime;
    }
}

void URRPoseOdomComponent::PreInitializePublisher(UROS2NodeComponent* InROS2Node, const FString& InTopicName)
{
    Super::PreInitializePublisher(InROS2Node, InTopicName);

    URRROS2OdomPublisher* odomPub = Cast<URRROS2OdomPublisher>(SensorPublisher);
    if (odomPub)
    {
        odomPub->bPublishOdomTf = bPublishOdomTf;
    }
}

void URRPoseOdomComponent::SetFrameIds(const FString& InFrameId, const FString& InChildFrameId)
{
    OdomData.Header.FrameId = FrameId = InFrameId;
    OdomData.ChildFrameId = ChildFrameId = InChildFrameId;
}

// todo separate ROS
void URRPoseOdomComponent::InitOdom()
{
    GaussianRNGPosition = std::normal_distribution<>{NoiseMeanPos, NoiseVariancePos};
    GaussianRNGRotation = std::normal_distribution<>{NoiseMeanRot, NoiseVarianceRot};

    AActor* owner = GetOwner();
    OdomData.Header.FrameId = FrameId;
    OdomData.ChildFrameId = ChildFrameId;

    if (OdomSource == EOdomSource::ENCODER)
    {    // odom source = encoder. Odom frame start from robot initial pose
        InitialTransform.SetTranslation(owner->GetActorLocation());
        InitialTransform.SetRotation(owner->GetActorQuat());
    }
    else
    {    // odom source = world. Odom frame start from world origin
        InitialTransform.SetTranslation(FVector::ZeroVector);
        InitialTransform.SetRotation(FQuat::Identity);
    }

    OdomData.Pose.Pose.Position = InitialTransform.GetTranslation();
    OdomData.Pose.Pose.Orientation = InitialTransform.GetRotation();

    PreviousTransform = InitialTransform;
    PreviousNoisyTransform = InitialTransform;

    // todo temporary hardcoded
    OdomData.Pose.Covariance[0] = 1e-05f;
    OdomData.Pose.Covariance[7] = 1e-05f;
    OdomData.Pose.Covariance[14] = 1e+12;
    OdomData.Pose.Covariance[21] = 1e+12;
    OdomData.Pose.Covariance[28] = 1e+12;
    OdomData.Pose.Covariance[35] = 1e-03f;

    OdomData.Twist.Covariance[0] = 1e-05f;
    OdomData.Twist.Covariance[7] = 1e-05f;
    OdomData.Twist.Covariance[14] = 1e+12;
    OdomData.Twist.Covariance[21] = 1e+12;
    OdomData.Twist.Covariance[28] = 1e+12;
    OdomData.Twist.Covariance[35] = 1e-03f;

    bIsOdomInitialized = true;
    LastUpdatedTime = UGameplayStatics::GetTimeSeconds(GetWorld());

    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("%s %s"), *FrameId, *OdomData.Header.FrameId);
}

void URRPoseOdomComponent::UpdateOdom(float InDeltaTime)
{
    if (!bIsOdomInitialized)
    {
        InitOdom();
    }

    if (InDeltaTime < 1e-9f)
    {
        return;
    }

    // time
    OdomData.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));

    // previous estimated data (with noise)
    FVector previousEstimatedPos = PreviousNoisyTransform.GetTranslation();
    FQuat previousEstimatedRot = PreviousNoisyTransform.GetRotation();

    AActor* owner = GetOwner();

    // position
    FVector pos = InitialTransform.GetRotation().UnrotateVector(owner->GetActorLocation() - InitialTransform.GetTranslation());
    FVector previousPos = PreviousTransform.GetTranslation();    // prev pos without noise
    PreviousTransform.SetTranslation(pos);
    pos += previousEstimatedPos - previousPos + WithNoise * FVector(GaussianRNGPosition(Gen), GaussianRNGPosition(Gen), 0);

    FRotator noiseRot = FRotator(0, 0, WithNoise * GaussianRNGRotation(Gen));
    FQuat rot = owner->GetActorQuat() * InitialTransform.GetRotation().Inverse();
    FQuat previousRot = PreviousTransform.GetRotation();
    PreviousTransform.SetRotation(rot);
    rot = noiseRot.Quaternion() * previousEstimatedRot * previousRot.Inverse() * rot;
    rot.Normalize();

    PreviousNoisyTransform.SetTranslation(pos);
    PreviousNoisyTransform.SetRotation(rot);

    OdomData.Pose.Pose.Position = pos + RootOffset.GetTranslation();
    OdomData.Pose.Pose.Orientation = rot;

    OdomData.Twist.Twist.Linear = OdomData.Pose.Pose.Orientation.UnrotateVector(pos - previousEstimatedPos) / InDeltaTime;
    OdomData.Twist.Twist.Angular =
        FMath::DegreesToRadians((rot * previousEstimatedRot.Inverse()).GetNormalized().Euler()) / InDeltaTime;

    OdomData.Pose.Pose.Orientation *= RootOffset.GetRotation();
}

FTransform URRPoseOdomComponent::GetOdomTF() const
{
    return FTransform(OdomData.Pose.Pose.Orientation, OdomData.Pose.Pose.Position);
}
