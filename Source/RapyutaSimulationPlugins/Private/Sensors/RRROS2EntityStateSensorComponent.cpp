// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2EntityStateSensorComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"

URRROS2EntityStateSensorComponent::URRROS2EntityStateSensorComponent()
{
    TopicName = TEXT("entity_state");
    MsgClass = UROS2EntityStateMsg::StaticClass();
}

void URRROS2EntityStateSensorComponent::BeginPlay()
{
    Super::BeginPlay();
    if (!URRGeneralUtils::GetRelativeTransform(ReferenceActorName, ReferenceActor, GetComponentTransform(), pastRelativeTransf))
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Reference Actor %s is not valid."), *ReferenceActorName);
    }
    pastTime = GetWorld()->GetGameState()->GetServerWorldTimeSeconds();
}

void URRROS2EntityStateSensorComponent::SetReferenceActorByName(const FString& InName)
{
    AActor* newReferenceActor = URRGeneralUtils::FindActorByName<AActor>(GetWorld(), InName);
    if (newReferenceActor)
    {
        const bool bNewReference = (ReferenceActor != newReferenceActor);
        ReferenceActor = newReferenceActor;
        ReferenceActorName = InName;
        if (bNewReference)
        {
            OnNewReferenceActorDetected.Broadcast(newReferenceActor);
        }
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("%s is not found"), *InName);
    }
}

void URRROS2EntityStateSensorComponent::SetReferenceActorByActor(AActor* InActor)
{
    const bool bNewReference = (ReferenceActor != InActor);
    ReferenceActor = InActor;
    ReferenceActorName = ReferenceActor->GetName();
    if (bNewReference)
    {
        OnNewReferenceActorDetected.Broadcast(InActor);
    }
}

FROSEntityState URRROS2EntityStateSensorComponent::GetROS2Data()
{
    return Data;
}

void URRROS2EntityStateSensorComponent::SensorUpdate()
{
    FTransform relativeTransf;
    if (!URRGeneralUtils::GetRelativeTransform(ReferenceActor, GetComponentTransform(), relativeTransf, true))
    {
        if (bIsValid)
        {
            // warning output once
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Reference Actor %s is not valid."), *ReferenceActorName);
        }
        bIsValid = false;
        return;
    }

    relativeTransf = URRConversionUtils::TransformUEToROS(OffsetTransform * relativeTransf);

    Data.Pose.Position = relativeTransf.GetTranslation() + RootOffset.GetTranslation();
    Data.Pose.Orientation = relativeTransf.GetRotation() * RootOffset.GetRotation();
    Data.ReferenceFrame = ReferenceActorName;

    // todo calc vel

        
    float currentTime = GetWorld()->GetGameState()->GetServerWorldTimeSeconds();
    float deltaTime = (currentTime - pastTime);
    FVector deltaVector = relativeTransf.GetTranslation() - pastRelativeTransf.GetTranslation();

    Data.Twist.Linear = deltaVector/deltaTime;
    Data.Twist.Angular = FVector::ZeroVector;

    pastRelativeTransf = relativeTransf;
    pastTime = currentTime;
    
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
