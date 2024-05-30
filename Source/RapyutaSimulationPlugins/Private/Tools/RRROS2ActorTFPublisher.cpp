// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ActorTFPublisher.h"

// rclUE
#include "ROS2ServiceServer.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"
#include "Core/RRUObjectUtils.h"

bool URRROS2ActorTFPublisher::InitializeWithROS2(UROS2NodeComponent* InROS2Node)
{
    bool result = Super::InitializeWithROS2(InROS2Node);

    if (result)
    {
        ROS2_CREATE_SERVICE_SERVER(
            InROS2Node, this, TriggerServiceName, UROS2SetBoolSrv::StaticClass(), &URRROS2ActorTFPublisher::TriggerPublishSrv);
    }
    return result;
}

void URRROS2ActorTFPublisher::TriggerPublishSrv(UROS2GenericSrv* Service)
{
    UROS2SetBoolSrv* triggerPublishService = Cast<UROS2SetBoolSrv>(Service);

    FROSSetBoolReq request;
    triggerPublishService->GetRequest(request);
    if (request.bData)
    {
        StartPublishTimer();
    }
    else
    {
        StopPublishTimer();
    }

    FROSSetBoolRes response;
    response.bSuccess = true;
    triggerPublishService->SetResponse(response);
}

void URRROS2ActorTFPublisher::SetReferenceActorByName(const FString& InName)
{
    auto newRefActor = URRGeneralUtils::FindActorByName<AActor>(GetWorld(), InName);
    if (newRefActor)
    {
        ReferenceActor = newRefActor;
        ReferenceActorName = InName;
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Reference Actor %s is not valid."), *ReferenceActorName);
    }
}

void URRROS2ActorTFPublisher::SetReferenceActorByActor(AActor* InActor)
{
    if (InActor)
    {
        ReferenceActor = InActor;
        ReferenceActorName = ReferenceActor->GetName();
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Reference Actor is not valid."));
    }
}

void URRROS2ActorTFPublisher::SetTargetActorByName(const FString& InName)
{
    auto newTargetActor = URRGeneralUtils::FindActorByName<AActor>(GetWorld(), InName);
    if (newTargetActor)
    {
        TargetActor = newTargetActor;
        TargetActorName = InName;
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Target Actor %s is not valid."), *ReferenceActorName);
    }
}

void URRROS2ActorTFPublisher::SetTargetActorByActor(AActor* InActor)
{
    if (InActor)
    {
        TargetActor = InActor;
        TargetActorName = TargetActor->GetName();
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Target Actor is not valid."));
    }
}

void URRROS2ActorTFPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (TargetActor == nullptr)
    {
        if (bIsValid)
        {
            // warning output once
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Target Actor %s is not valid."), *TargetActor->GetName());
        }
        bIsValid = false;
        return;
    }

    if (!URRGeneralUtils::GetRelativeTransform(ReferenceActor, TargetActor->GetTransform(), TF, true))
    {
        if (bIsValid)
        {
            // warning output once
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Reference Actor %s is not valid."), *ReferenceActorName);
        }
        bIsValid = false;
        return;
    }

    bIsValid = true;
    Super::UpdateMessage(InMessage);
}
