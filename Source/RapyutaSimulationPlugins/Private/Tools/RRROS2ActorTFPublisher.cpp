// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ActorTFPublisher.h"

void URRROS2ActorTFPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    // register delegates to node
    FServiceCallback TriggerPublishSrvCallback; 
    TriggerPublishSrvCallback.BindUObject(this, &URRROS2ActorTFPublisher::TriggerPublishSrv);
    InROS2Node->AddService(TriggerServiceName, UROS2SetBoolSrv::StaticClass(), TriggerPublishSrvCallback);
}

void URRROS2ActorTFPublisher::TriggerPublishSrv(UROS2GenericSrv* Service)
{
    UROS2SetBoolSrv* TriggerPublishService = Cast<UROS2SetBoolSrv>(Service);

    FROSSetBool_Request Request;
    TriggerPublishService->GetRequest(Request);
    bPublish = Request.data;

    FROSSetBool_Response Response;
    Response.success = true;
    TriggerPublishService->SetResponse(Response);
}

void URRROS2ActorTFPublisher::BeginPlay()
{
    SetReferenceActor(ReferenceActorName);
    SetTargetActor(TargetActorName);
    Super::BeginPlay();
}

void URRROS2ActorTFPublisher::SetReferenceActor(const FString& InName)
{
    ReferenceActor = URRGeneralUtils::GetActorByName(GetWorld(), InName);
    ReferenceActorName = InName;
}

void URRROS2ActorTFPublisher::SetTargetActor(const FString& InName)
{
    TargetActor = URRGeneralUtils::GetActorByName(GetWorld(), InName);
    TargetActorName = InName;
}

void URRROS2ActorTFPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if(TargetActor == nullptr)
    {
        if (bIsValid)
        {
            //warning output once
            UE_LOG(LogRapyutaCore, Warning, TEXT("Target Actor %s is not valid."), *TargetActor->GetName());
        }
        bIsValid = false;
        return;
    }

    if (!URRGeneralUtils::GetRelativeTransform(ReferenceActorName, ReferenceActor, TargetActor->GetTransform(), TF)) 
    {
        if (bIsValid)
        {
            //warning output once
            UE_LOG(LogRapyutaCore, Warning, TEXT("Reference Actor %s is not valid."), *ReferenceActorName);
        }
        bIsValid = false;
        return;
    }

    bIsValid = true;
    Super::UpdateMessage(InMessage);
}
