// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2EntityStateSensorComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"

URRROS2EntityStateSensorComponent::URRROS2EntityStateSensorComponent()
{
    SensorPublisherClass = URRROS2EntityStatePublisher::StaticClass();
}

void URRROS2EntityStateSensorComponent::BeginPlay()
{
    Super::BeginPlay();
}

void URRROS2EntityStateSensorComponent::SetReferenceActorByName(const FString& InName)
{
    ReferenceActor = URRUObjectUtils::FindActorByName<AActor>(GetWorld(), InName);
    ReferenceActorName = InName;
}

void URRROS2EntityStateSensorComponent::SetReferenceActorByActor(AActor* InActor)
{
    ReferenceActor = InActor;
    ReferenceActorName = ReferenceActor->GetName();
}

FROSEntityState URRROS2EntityStateSensorComponent::GetROS2Data()
{
    return Data;
}

void URRROS2EntityStateSensorComponent::SensorUpdate()
{
    FTransform relativeTransf;
    if (!URRGeneralUtils::GetRelativeTransform(ReferenceActorName, ReferenceActor, GetComponentTransform(), relativeTransf))
    {
        // Print error for once if this is the first invalid scan data since the last valid one
        if (bLastScanDataValid)
        {
            // warning output once
            UE_LOG(LogRapyutaCore, Warning, TEXT("Reference Actor %s is not valid."), *ReferenceActorName);
        }
        bLastScanDataValid = false;
        return;
    }

    relativeTransf = URRConversionUtils::TransformUEToROS(relativeTransf);

    Data.Pose.Position = relativeTransf.GetTranslation() + RootOffset.GetTranslation();
    Data.Pose.Orientation = relativeTransf.GetRotation() * RootOffset.GetRotation();
    Data.ReferenceFrame = ReferenceActorName;

    // todo calc vel
    Data.Twist.Linear = FVector::ZeroVector;
    Data.Twist.Angular = FVector::ZeroVector;

    bLastScanDataValid = true;
}

void URRROS2EntityStateSensorComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2EntityStateMsg>(InMessage)->SetMsg(GetROS2Data());
}

void URRROS2EntityStateSensorComponent::SetRootOffset(const FTransform& InRootOffset)
{
    RootOffset = URRConversionUtils::TransformUEToROS(InRootOffset);
}
