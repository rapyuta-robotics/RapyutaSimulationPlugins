// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotPoseSensorManager.h"

// UE
#include "Net/UnrealNetwork.h"

URRRobotPoseSensorManager::URRRobotPoseSensorManager()
{
    MapOriginPoseSensor = CreateDefaultSubobject<URRROS2EntityStateSensorComponent>(TEXT("MapTFPublisher"));
    MapOriginPoseSensor->SetupAttachment(this);

    // default settings
    TopicName = TEXT("ue_ros/model_state");
    PublicationFrequencyHz = 30;
    FrameId = TEXT("base_footprint");
    MapOriginPoseSensor->TopicName = TEXT("ue_ros/map_origin_entity_state");
    MapOriginPoseSensor->PublicationFrequencyHz = 10;
    MapOriginPoseSensor->FrameId = MapFrameId;
}

void URRRobotPoseSensorManager::OnComponentCreated()
{
    Super::OnComponentCreated();
    SetIsReplicated(true);
}

void URRRobotPoseSensorManager::InitalizeWithROS2(AROS2Node* InROS2Node,
                                                  const FString& InPublisherName,
                                                  const FString& InTopicName,
                                                  const TEnumAsByte<UROS2QoS> InQoS)
{
    Super::InitalizeWithROS2(InROS2Node, InPublisherName, InTopicName, InQoS);
    MapOriginPoseSensor->InitalizeWithROS2(InROS2Node);
    ServerSimState = CastChecked<ASimulationState>(UGameplayStatics::GetActorOfClass(GetWorld(), ASimulationState::StaticClass()));
}

void URRRobotPoseSensorManager::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URRRobotPoseSensorManager, ServerSimState);
    DOREPLIFETIME(URRRobotPoseSensorManager, ReferenceTag);
    DOREPLIFETIME(URRRobotPoseSensorManager, RefActorSelectMode);
    DOREPLIFETIME(URRRobotPoseSensorManager, MapFrameId);
    DOREPLIFETIME(URRRobotPoseSensorManager, MapOriginPoseSensor);
}

void URRRobotPoseSensorManager::UpdateReferenceActorWithTag()
{
    if (ReferenceTag.IsEmpty())
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[URRRobotPoseSensorManager] RefActorSelectMode is AUTO but reference tag %s is empty."),
               *ReferenceTag);
        return;
    }

    if (ServerSimState == nullptr)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[URRRobotPoseSensorManager]ServerSimState is null."));
        ServerSimState =
            CastChecked<ASimulationState>(UGameplayStatics::GetActorOfClass(GetWorld(), ASimulationState::StaticClass()));
        return;
    }

    // search object with tag and
    // do not use GetAllActorsWithTag since it is slow.
    // set nearest actor in z axis as ReferenceActor
    AActor* nearestActor = nullptr;
    const TArray<AActor*> actors = ServerSimState->EntitiesWithTag.FindRef(FName(ReferenceTag)).Actors;
    if (actors.Num() > 0)
    {
        float sensorPoseZ = GetComponentTransform().GetTranslation().Z;
        int32 nearestActorIndex = 0;

        float minDiffZ = FMath::Abs(sensorPoseZ - actors[0]->GetActorLocation().Z);
        for (int32 index = 1; index < actors.Num(); ++index)
        {
            float diffZ = FMath::Abs(sensorPoseZ - actors[index]->GetActorLocation().Z);
            if (minDiffZ > diffZ)
            {
                minDiffZ = diffZ;
                nearestActorIndex = index;
            }
        }
        nearestActor = actors[nearestActorIndex];
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[URRRobotPoseSensorManager] ServerSimState's EntitiesWithTag for [%s] tag is empty"),
               *ReferenceTag);
    }

    if (nearestActor != nullptr)
    {
        if (nearestActor != ReferenceActor)
        {
            SetReferenceActorByActor(nearestActor);
            MapOriginPoseSensor->DetachFromComponent(FDetachmentTransformRules::KeepRelativeTransform);
            MapOriginPoseSensor->AttachToComponent(nearestActor->GetRootComponent(),
                                                   FAttachmentTransformRules::KeepRelativeTransform);
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[URRRobotPoseSensorManager] No Actor with [%s] tag found in ServerSimState's "
                    "EntitiesWithTag."),
               *ReferenceTag);
    }
}

void URRRobotPoseSensorManager::SensorUpdate()
{
    if (RefActorSelectMode == ERRRefActorSelectMode::AUTO)
    {
        UpdateReferenceActorWithTag();
    }
    Super::SensorUpdate();
}
