// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2OverlapSensorComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"

void URRROS2OverlapSensorComponent::BeginPlay()
{
    Super::BeginPlay();

    // default target is owner
    if (TargetObjects.Num() == 0)
    {
        TargetObjects.Add(GetOwner());
    }

    BoundCallbacks(TargetObjects);
}

void URRROS2OverlapSensorComponent::BoundCallbacks(const TArray<UObject*> InTargetObjects)
{
    for (const auto target : InTargetObjects)
    {
        auto primitiveComp = Cast<UPrimitiveComponent>(target);
        if (primitiveComp)
        {
            primitiveComp->OnComponentBeginOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnComponentBeginOverlap);
            primitiveComp->OnComponentEndOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnComponentEndOverlap);
            return;
        }

        auto actor = Cast<AActor>(target);
        if (actor)
        {
            actor->OnActorBeginOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnActorBeginOverlap);
            actor->OnActorEndOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnActorEndOverlap);
            return;
        }

        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("TargetObject must be child class of UPrimitiveComponent or AActor"))
    }
}

void URRROS2OverlapSensorComponent::OnOverlap(AActor* OverlappedActor,
                                              AActor* OtherActor,
                                              UPrimitiveComponent* OtherComp,
                                              const bool InBegin,
                                              const FString& Name)
{
    Data.bBegin = InBegin;
    Data.SelfName = Name.IsEmpty() ? OverlappedActor->GetName() : Name;
    ;
    Data.OtherActorName = OtherActor->GetName();
    if (IsIgnore(OverlappedActor, OtherActor, OtherComp))
    {
        return;
    }

    SetROS2Msg(SensorPublisher->TopicMessage);
    SensorPublisher->Publish();
    Data = FROSOverlapEvent();
}

void URRROS2OverlapSensorComponent::OnComponentOverlap(UPrimitiveComponent* OverlappedComponent,
                                                       AActor* OtherActor,
                                                       UPrimitiveComponent* OtherComp,
                                                       int32 OtherBodyIndex,
                                                       const bool InBegin)
{
    Data.OtherComponentName = OtherComp->GetName();
    Data.OtherBodyIndex = OtherBodyIndex;
    OnOverlap(OverlappedComponent->GetOwner(), OtherActor, OtherComp, InBegin, OverlappedComponent->GetName());
}

void URRROS2OverlapSensorComponent::OnComponentBeginOverlap(UPrimitiveComponent* OverlappedComponent,
                                                            AActor* OtherActor,
                                                            UPrimitiveComponent* OtherComp,
                                                            int32 OtherBodyIndex,
                                                            bool bFromSweep,
                                                            const FHitResult& SweepResult)
{
    Data.bFromSweep = bFromSweep;
    Data.HitResult = URRConversionUtils::HitResultUEToROS(SweepResult);
    OnComponentOverlap(OverlappedComponent, OtherActor, OtherComp, OtherBodyIndex, true);
}

void URRROS2OverlapSensorComponent::OnComponentEndOverlap(UPrimitiveComponent* OverlappedComponent,
                                                          AActor* OtherActor,
                                                          UPrimitiveComponent* OtherComp,
                                                          int32 OtherBodyIndex)
{
    OnComponentOverlap(OverlappedComponent, OtherActor, OtherComp, OtherBodyIndex, false);
}

void URRROS2OverlapSensorComponent::OnActorBeginOverlap(AActor* OverlappedActor, AActor* OtherActor)
{
    OnOverlap(OverlappedActor, OtherActor, nullptr, true);
}

void URRROS2OverlapSensorComponent::OnActorEndOverlap(AActor* OverlappedActor, AActor* OtherActor)
{
    OnOverlap(OverlappedActor, OtherActor, nullptr, false);
}

void URRROS2OverlapSensorComponent::SensorUpdate()
{
    bIsValid = true;
}

void URRROS2OverlapSensorComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2OverlapEventMsg>(InMessage)->SetMsg(Data);
}

bool URRROS2OverlapSensorComponent::IsIgnore(AActor* SelfActor, AActor* OtherActor, UPrimitiveComponent* OtherComp)
{
    bool result = false;
    if ((bIgnoreSelf && SelfActor == OtherActor) || IgnoreList.Contains(OtherActor) || IgnoreList.Contains(OtherComp))
    {
        result = true;
    }

    return result;
}
