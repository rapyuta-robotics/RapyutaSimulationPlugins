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
        if (bIsValid)
        {
            // warning output once
            UE_LOG(LogRapyutaCore, Warning, TEXT("Reference Actor %s is not valid."), *ReferenceActorName);
        }
        bIsValid = false;
        return;
    }

    relativeTransf = URRConversionUtils::TransformUEToROS(relativeTransf);

    Data.pose_position_x = relativeTransf.GetTranslation().X + RootOffset.GetTranslation().X;
    Data.pose_position_y = relativeTransf.GetTranslation().Y + RootOffset.GetTranslation().Y;
    Data.pose_position_z = relativeTransf.GetTranslation().Z + RootOffset.GetTranslation().Z;
    Data.pose_orientation = relativeTransf.GetRotation() * RootOffset.GetRotation();
    Data.reference_frame = ReferenceActorName;

    // todo calc vel
    Data.twist_linear = FVector::ZeroVector;
    Data.twist_angular = FVector::ZeroVector;

    bIsValid = true;
}

void URRROS2EntityStateSensorComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2EntityStateMsg>(InMessage)->SetMsg(GetROS2Data());
}

void URRROS2EntityStateSensorComponent::SetRootOffset(const FTransform& InRootOffset)
{
    RootOffset = URRConversionUtils::TransformUEToROS(InRootOffset);
}
