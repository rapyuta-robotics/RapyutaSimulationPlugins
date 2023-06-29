// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2OverlapSensorComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"

URRROS2OverlapSensorComponent::URRROS2OverlapSensorComponent()
{
    // collisions publisher
    TopicName = TEXT("overlaps");
    MsgClass = UROS2OverlapsMsg::StaticClass();
    PublicationFrequencyHz = 10;

    EventTopicName = TEXT("overlap_event");
}

void URRROS2OverlapSensorComponent::InitalizeWithROS2(UROS2NodeComponent* InROS2Node,
                                                      const FString& InPublisherName,
                                                      const FString& InTopicName,
                                                      const UROS2QoS InQoS)
{
    Super::InitalizeWithROS2(InROS2Node, InPublisherName, InTopicName, InQoS);
    EventPublisher =
        InROS2Node->CreatePublisher(EventTopicName, UROS2Publisher::StaticClass(), UROS2OverlapEventMsg::StaticClass());
}

void URRROS2OverlapSensorComponent::BeginPlay()
{
    Super::BeginPlay();

    // default target is owner
    if (TargetObjects.Num() == 0)
    {
        TargetObjects.Add(GetOwner());
    }

    // Initialize Overlapping struct
    for (const auto target : TargetObjects)
    {
        AddTarget(target);
    }
}

void URRROS2OverlapSensorComponent::AddTarget(UObject* InTargetObject)
{
    Overlaps.Targets.Add(InTargetObject->GetName());
    BindCallback(InTargetObject);
}

void URRROS2OverlapSensorComponent::BindCallback(UObject* InTargetObject)
{
    auto primitiveComp = Cast<UPrimitiveComponent>(InTargetObject);
    if (primitiveComp)
    {
        primitiveComp->OnComponentBeginOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnTargetComponentBeginOverlap);
        primitiveComp->OnComponentEndOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnTargetComponentEndOverlap);
        return;
    }

    auto actor = Cast<AActor>(InTargetObject);
    if (actor)
    {
        actor->OnActorBeginOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnTargetActorBeginOverlap);
        actor->OnActorEndOverlap.AddDynamic(this, &URRROS2OverlapSensorComponent::OnTargetActorEndOverlap);
        return;
    }

    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("TargetObject must be child class of UPrimitiveComponent or AActor"))
}

void URRROS2OverlapSensorComponent::OnOverlap(AActor* OverlappedActor,
                                              AActor* OtherActor,
                                              UPrimitiveComponent* OtherComp,
                                              const bool InBegin,
                                              const FString& Name)
{
    if (IsIgnore(OverlappedActor, OtherActor, OtherComp))
    {
        return;
    }

    Data.bBegin = InBegin;
    Data.SelfName = Name.IsEmpty() ? OverlappedActor->GetName() : Name;
    Data.OtherActorName = OtherActor->GetName();

    CastChecked<UROS2OverlapEventMsg>(EventPublisher->TopicMessage)->SetMsg(Data);
    EventPublisher->Publish();
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

void URRROS2OverlapSensorComponent::OnTargetComponentBeginOverlap(UPrimitiveComponent* OverlappedComponent,
                                                                  AActor* OtherActor,
                                                                  UPrimitiveComponent* OtherComp,
                                                                  int32 OtherBodyIndex,
                                                                  bool bFromSweep,
                                                                  const FHitResult& SweepResult)
{
    Data.bFromSweep = bFromSweep;
    Data.SweepResult = URRConversionUtils::HitResultUEToROS(SweepResult);
    OnComponentOverlap(OverlappedComponent, OtherActor, OtherComp, OtherBodyIndex, true);
}

void URRROS2OverlapSensorComponent::OnTargetComponentEndOverlap(UPrimitiveComponent* OverlappedComponent,
                                                                AActor* OtherActor,
                                                                UPrimitiveComponent* OtherComp,
                                                                int32 OtherBodyIndex)
{
    OnComponentOverlap(OverlappedComponent, OtherActor, OtherComp, OtherBodyIndex, false);
}

void URRROS2OverlapSensorComponent::OnTargetActorBeginOverlap(AActor* OverlappedActor, AActor* OtherActor)
{
    OnOverlap(OverlappedActor, OtherActor, nullptr, true);
}

void URRROS2OverlapSensorComponent::OnTargetActorEndOverlap(AActor* OverlappedActor, AActor* OtherActor)
{
    OnOverlap(OverlappedActor, OtherActor, nullptr, false);
}

void URRROS2OverlapSensorComponent::SensorUpdate()
{
    bIsValid = true;
    // Initialize Overlapping struct
    for (const auto target : TargetObjects)
    {
        FROSOverlappingObjects overlappingObjects;
        TArray<UPrimitiveComponent*> overlappingComponents;
        TArray<AActor*> overlappingActors;

        AActor* actor;
        auto primitiveComp = Cast<UPrimitiveComponent>(target);
        if (primitiveComp)
        {
            primitiveComp->GetOverlappingActors(overlappingActors);
            primitiveComp->GetOverlappingComponents(overlappingComponents);

            actor = primitiveComp->GetOwner();
        }
        else
        {
            actor = Cast<AActor>(target);
            if (actor)
            {
                actor->GetOverlappingActors(overlappingActors);
                actor->GetOverlappingComponents(overlappingComponents);
            }
        }

        for (const auto oactor : overlappingActors)
        {
            if (!IsIgnore(actor, oactor, nullptr))
            {
                overlappingObjects.Actors.Add(oactor->GetName());
            }
        }
        for (const auto comp : overlappingComponents)
        {
            if (!IsIgnore(actor, comp->GetOwner(), comp))
            {
                overlappingObjects.Components.Add(comp->GetName());
            }
        }

        Overlaps.Overlaps.Empty();
        Overlaps.Overlaps.Add(overlappingObjects);
    }
}

void URRROS2OverlapSensorComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2OverlapsMsg>(InMessage)->SetMsg(Overlaps);
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
