// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2HitSensorComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

void URRROS2HitSensorComponent::BeginPlay()
{
    Super::BeginPlay();

    // default target is owner
    if (TargetObjects.Num() == 0)
    {
        TargetObjects.Add(GetOwner());
    }

    for (const auto target : TargetObjects)
    {
        BindCallback(target);
    }
}

void URRROS2HitSensorComponent::BindCallback(UObject* InTargetObject)
{
    auto primitiveComp = Cast<UPrimitiveComponent>(InTargetObject);
    if (primitiveComp)
    {
        primitiveComp->OnComponentHit.AddDynamic(this, &URRROS2HitSensorComponent::OnTargetComponentHit);
        return;
    }

    auto actor = Cast<AActor>(InTargetObject);
    if (actor)
    {
        actor->OnActorHit.AddDynamic(this, &URRROS2HitSensorComponent::OnTargetActorHit);
        return;
    }

    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("TargetObject must be child class of UPrimitiveComponent or AActor"))
}

void URRROS2HitSensorComponent::OnTargetComponentHit(UPrimitiveComponent* HitComp,
                                                     AActor* OtherActor,
                                                     UPrimitiveComponent* OtherComp,
                                                     FVector NormalImpulse,
                                                     const FHitResult& Hit)
{
    OnHit(HitComp->GetOwner(), OtherActor, NormalImpulse, Hit, HitComp->GetName());
}

void URRROS2HitSensorComponent::OnTargetActorHit(AActor* SelfActor,
                                                 AActor* OtherActor,
                                                 FVector NormalImpulse,
                                                 const FHitResult& Hit)
{
    OnHit(SelfActor, OtherActor, NormalImpulse, Hit);
}

void URRROS2HitSensorComponent::OnHit(AActor* SelfActor,
                                      AActor* OtherActor,
                                      FVector NormalImpulse,
                                      const FHitResult& Hit,
                                      const FString& Name)
{
    Data = FROSHitEvent();
    Data.SelfName = Name.IsEmpty() ? SelfActor->GetName() : Name;
    Data.OtherActorName = OtherActor->GetName();
    Data.NormalImpluse = NormalImpulse;
    Data.HitResult = URRConversionUtils::HitResultUEToROS(Hit);
    Data.OtherComponentName = Data.HitResult.ComponentName;
    if (IsIgnore(SelfActor, OtherActor, Hit.Component.Get()))
    {
        return;
    }

    SetROS2Msg(SensorPublisher->TopicMessage);
    SensorPublisher->Publish();
}

void URRROS2HitSensorComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2HitEventMsg>(InMessage)->SetMsg(Data);
}

bool URRROS2HitSensorComponent::IsIgnore(AActor* SelfActor, AActor* OtherActor, UPrimitiveComponent* OtherComp)
{
    bool result = false;
    if ((bIgnoreSelf && SelfActor == OtherActor) || IgnoreList.Contains(OtherActor) || IgnoreList.Contains(OtherComp))
    {
        result = true;
    }

    return result;
}
