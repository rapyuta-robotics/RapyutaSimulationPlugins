// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2HitSensorComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"

void URRROS2HitSensorComponent::BeginPlay()
{
    Super::BeginPlay();
    BoundCallbacks(TargetObject);
}

void URRROS2HitSensorComponent::BoundCallbacks(UObject* InTargetObject)
{
    auto primitiveComp = Cast<UPrimitiveComponent>(InTargetObject);
    if (primitiveComp)
    {
        primitiveComp->OnComponentHit.AddDynamic(this, &URRROS2HitSensorComponent::OnComponentHit);
        return;
    }

    auto actor = Cast<AActor>(InTargetObject);
    if (actor)
    {
        actor->OnActorHit.AddDynamic(this, &URRROS2HitSensorComponent::OnActorHit);
        return;
    }

    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("TargetObject must be child class of UPrimitiveComponent or AActor"))
}

void URRROS2HitSensorComponent::OnComponentHit(UPrimitiveComponent* HitComp,
                                               AActor* OtherActor,
                                               UPrimitiveComponent* OtherComp,
                                               FVector NormalImpulse,
                                               const FHitResult& Hit)
{
    Data.SelfName = HitComp->GetName();
    Data.OtherActorName = OtherActor->GetName();
    Data.NormalImpluse = NormalImpulse;
    Data.HitResult = URRConversionUtils::HitResultUEToROS(Hit);
    Data.OtherComponentName = OtherComp->GetName();
    SetROS2Msg(SensorPublisher->TopicMessage);
    SensorPublisher->Publish();
}

void URRROS2HitSensorComponent::OnActorHit(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& Hit)
{
    Data.SelfName = SelfActor->GetName();
    Data.OtherActorName = OtherActor->GetName();
    Data.NormalImpluse = NormalImpulse;
    Data.HitResult = URRConversionUtils::HitResultUEToROS(Hit);
    Data.OtherComponentName = Data.HitResult.ComponentName;
    SetROS2Msg(SensorPublisher->TopicMessage);
    SensorPublisher->Publish();
}

void URRROS2HitSensorComponent::SensorUpdate()
{
    bIsValid = true;
}

void URRROS2HitSensorComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2HitEventMsg>(InMessage)->SetMsg(Data);
}
