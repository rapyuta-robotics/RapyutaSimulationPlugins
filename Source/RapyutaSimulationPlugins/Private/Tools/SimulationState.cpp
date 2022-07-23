// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/SimulationState.h"

// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

// rclUE
#include "Srvs/ROS2AttachSrv.h"
#include "Srvs/ROS2DeleteEntitySrv.h"
#include "Srvs/ROS2GetEntityStateSrv.h"
#include "Srvs/ROS2SetEntityStateSrv.h"
#include "Srvs/ROS2SpawnEntitiesSrv.h"
#include "Srvs/ROS2SpawnEntitySrv.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"
#include "Tools/ROS2Spawnable.h"

ASimulationState::ASimulationState()
{
    bReplicates = true;
    PrimaryActorTick.bCanEverTick = true;
    bAlwaysRelevant = true;
}

void ASimulationState::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(ASimulationState, EntityList);
    DOREPLIFETIME(ASimulationState, SpawnableEntityInfoList);
}

void ASimulationState::InitEntities()
{
#if RAPYUTA_SIM_DEBUG
    TArray<AActor*> allActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), allActors);
    UE_LOG(LogTemp, Warning, TEXT("Found %d actors in the scene"), allActors.Num());
#endif
    // add all actors
    for (TActorIterator<AActor> It(GetWorld(), AActor::StaticClass()); It; ++It)
    {
        AActor* actor = *It;
        AddEntity(actor);
    }

    // NOTE: [SpawnableEntityInfoList] is a TArray<> thus replicatable, which is not supported for [SpawnableEntities] as a TMap
    GetWorld()->GetTimerManager().SetTimer(TimerHandle, this, &ASimulationState::GetSpawnableEntityInfoList, 1.0f, true);
}

void ASimulationState::AddEntity(AActor* InEntity)
{
    if (false == IsValid(InEntity))
    {
        return;
    }

    GetSpawnableEntityInfoList();
    Entities.Emplace(InEntity->GetName(), InEntity);
    EntityList.Emplace(InEntity);
    for (auto& tag : InEntity->Tags)
    {
        if (EntitiesWithTag.Contains(tag))
        {
            EntitiesWithTag[tag].Actors.Emplace(InEntity);
        }
        else
        {
            FRREntities actors;
            actors.Actors.Emplace(InEntity);
            EntitiesWithTag.Emplace(tag, MoveTemp(actors));
        }
    }
}

// Work around to replicating Entities and EntitiesWithTag since TMaps cannot be replicated
void ASimulationState::OnRep_Entity()
{
    for (AActor* entity : EntityList)
    {
        if (false == IsValid(entity))
        {
            continue;
        }

        if (!Entities.Contains(entity->GetName()))
        {
            UROS2Spawnable* rosSpawnParameters = entity->FindComponentByClass<UROS2Spawnable>();
            if (rosSpawnParameters)
            {
                Entities.Emplace(rosSpawnParameters->GetName(), entity);
            }
            else
            {
                Entities.Emplace(entity->GetName(), entity);
            }
        }

        for (const auto& tag : entity->Tags)
        {
            AddTaggedEntities(entity, tag);
        }

        UROS2Spawnable* EntitySpawnParam = entity->FindComponentByClass<UROS2Spawnable>();
        if (EntitySpawnParam)
        {
            entity->Rename(*EntitySpawnParam->GetName());
            for (const auto& tag : EntitySpawnParam->ActorTags)
            {
                AddTaggedEntities(entity, FName(tag));
            }
        }
    }
}

void ASimulationState::OnRep_SpawnableEntity()
{
    for (const auto& entityInfo : SpawnableEntityInfoList)
    {
        SpawnableEntityTypes.Emplace(entityInfo.EntityTypeName, entityInfo.EntityClass);
    }
}

void ASimulationState::AddTaggedEntities(AActor* Entity, const FName& InTag)
{
    if (EntitiesWithTag.Contains(InTag))
    {
        // Check if Actor in EntitiesWithTag

        bool ToEmplace = true;
        for (AActor* TaggedActor : EntitiesWithTag[InTag].Actors)
        {
            if (TaggedActor->GetName() == Entity->GetName())
            {
                ToEmplace = false;
            }
        }
        if (ToEmplace)
        {
            EntitiesWithTag[InTag].Actors.Emplace(Entity);
        }
    }
    else
    {
        FRREntities actors;
        actors.Actors.Emplace(Entity);
        EntitiesWithTag.Emplace(InTag, actors);
    }
}

void ASimulationState::AddSpawnableEntityTypes(TMap<FString, TSubclassOf<AActor>> InSpawnableEntityTypes)
{
    for (auto& elem : InSpawnableEntityTypes)
    {
        SpawnableEntityInfoList.Emplace(FRREntityInfo(elem));
        SpawnableEntityTypes.Emplace(MoveTemp(elem.Key), MoveTemp(elem.Value));
    }
}

void ASimulationState::GetSpawnableEntityInfoList()
{
    for (auto& elem : SpawnableEntityTypes)
    {
        SpawnableEntityInfoList.Emplace(FRREntityInfo(elem));
    }
    if (SpawnableEntityInfoList.Num() > 0)
    {
        GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
    }
}

bool ASimulationState::ServerCheckSetEntityStateRequest(const FROSSetEntityState_Request& InRequest)
{
    if (PreviousSetEntityStateRequest.state_name == InRequest.state_name &&
        PreviousSetEntityStateRequest.state_reference_frame == InRequest.state_reference_frame &&
        PreviousSetEntityStateRequest.state_pose_position_x == InRequest.state_pose_position_x &&
        PreviousSetEntityStateRequest.state_pose_position_y == InRequest.state_pose_position_y &&
        PreviousSetEntityStateRequest.state_pose_position_z == InRequest.state_pose_position_z &&
        PreviousSetEntityStateRequest.state_pose_orientation == InRequest.state_pose_orientation)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void ASimulationState::ServerSetEntityState(const FROSSetEntityState_Request& InRequest)
{
    if (ServerCheckSetEntityStateRequest(InRequest))
    {
        FVector pos(InRequest.state_pose_position_x, InRequest.state_pose_position_y, InRequest.state_pose_position_z);
        FTransform relativeTransf(InRequest.state_pose_orientation, pos);
        relativeTransf = URRConversionUtils::TransformROSToUE(relativeTransf);
        FTransform worldTransf;
        URRGeneralUtils::GetWorldTransform(
            InRequest.state_reference_frame,
            Entities.Contains(InRequest.state_reference_frame) ? Entities[InRequest.state_reference_frame] : nullptr,
            relativeTransf,
            worldTransf);
        Entities[InRequest.state_name]->SetActorTransform(worldTransf);
    }

    PreviousSetEntityStateRequest = InRequest;
}

bool ASimulationState::ServerCheckAttachRequest(const FROSAttach_Request& InRequest)
{
    return ((PreviousAttachRequest.name1 != InRequest.name1) || (PreviousAttachRequest.name2 != InRequest.name2));
}

void ASimulationState::ServerAttach(const FROSAttach_Request& InRequest)
{
    if (ServerCheckAttachRequest(InRequest))
    {
        AActor* entity1 = Entities[InRequest.name1];
        AActor* entity2 = Entities[InRequest.name2];

        if (!entity2->IsAttachedTo(entity1))
        {
            entity2->AttachToActor(entity1, FAttachmentTransformRules::KeepWorldTransform);
        }
        else
        {
            entity2->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("Entity %s and/or %s not exit or not under SimulationState Actor control. Please call AddEntity to make Actors "
                    "under SimulationState control."),
               *InRequest.name1,
               *InRequest.name2);
    }

    PreviousAttachRequest = InRequest;
}

bool ASimulationState::ServerCheckSpawnRequest(const FROSSpawnEntityRequest& InRequest)
{
    if (PreviousSpawnRequest.Xml == InRequest.Xml && PreviousSpawnRequest.RobotNamespace == InRequest.RobotNamespace &&
        PreviousSpawnRequest.StateName == InRequest.StateName &&
        PreviousSpawnRequest.StatePosePositionX == InRequest.StatePosePositionX &&
        PreviousSpawnRequest.StatePosePositionY == InRequest.StatePosePositionY &&
        PreviousSpawnRequest.StatePosePositionZ == InRequest.StatePosePositionZ &&
        PreviousSpawnRequest.StatePoseOrientation == InRequest.StatePoseOrientation &&
        PreviousSpawnRequest.StateReferenceFrame == InRequest.StateReferenceFrame)
    {
        return false;
    }
    else
    {
        return true;
    }
}

AActor* ASimulationState::ServerSpawnEntity(const FROSSpawnEntityRequest& InROSSpawnRequest,
                                            const TSubclassOf<AActor>& InEntityClass,
                                            const FTransform& InEntityTransform)
{
    // SpawnActorDeferred to set parameters beforehand
    AActor* newEntity = GetWorld()->SpawnActorDeferred<AActor>(InEntityClass, InEntityTransform);
    if (newEntity == nullptr)
    {
        return nullptr;
    }

    // Wrap [InROSSpawnRequest] into a ROS2 spawnable component
    UROS2Spawnable* spawnableComponent = NewObject<UROS2Spawnable>(newEntity, TEXT("ROS2 Spawn Parameters"));
    spawnableComponent->RegisterComponent();
    spawnableComponent->InitializeParameters(InROSSpawnRequest);
    spawnableComponent->SetIsReplicated(true);

    newEntity->AddInstanceComponent(spawnableComponent);
    newEntity->Rename(*InROSSpawnRequest.StateName);
    newEntity->SetReplicates(true);
    // Needs to be set to relevant otherwise it won't consistantly replicate
    newEntity->bAlwaysRelevant = true;
#if WITH_EDITOR
    newEntity->SetActorLabel(*InROSSpawnRequest.StateName);
#endif

    // Add tags
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[%s] tag from spawn spawn request %s"),
           *newEntity->GetName(),
           *FString::Join(InROSSpawnRequest.Tags, TEXT(",")));
    for (const auto& tag : InROSSpawnRequest.Tags)
    {
        newEntity->Tags.Emplace(tag);
        spawnableComponent->AddTag(tag);
    }

    // Finish spawning Entity
    UGameplayStatics::FinishSpawningActor(newEntity, InEntityTransform);

    // Add to [Entities]
    AddEntity(newEntity);
    return newEntity;
}

AActor* ASimulationState::ServerSpawnEntity(const FROSSpawnEntityRequest& InRequest)
{
    AActor* newEntity = nullptr;
    if (ServerCheckSpawnRequest(InRequest))
    {
        const FString& entityModelName = InRequest.Xml;
        const FString& entityName = InRequest.StateName;
        verify(false == entityName.IsEmpty());
        if (nullptr == URRUObjectUtils::FindActorByName<AActor>(GetWorld(), entityName))
        {
            // Calculate to-be-spawned entity's [world transf]
            FVector relLocation(InRequest.StatePosePositionX, InRequest.StatePosePositionY, InRequest.StatePosePositionZ);
            FTransform relativeTransf =
                URRConversionUtils::TransformROSToUE(FTransform(InRequest.StatePoseOrientation, relLocation));
            FTransform worldTransf;
            URRGeneralUtils::GetWorldTransform(
                InRequest.StateReferenceFrame, Entities.FindRef(InRequest.StateReferenceFrame), relativeTransf, worldTransf);
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("Spawning Entity of model [%s] as [%s] to pose: %s"),
                   *entityModelName,
                   *entityName,
                   *worldTransf.ToString());

            // Spawn entity
            newEntity = ServerSpawnEntity(InRequest, SpawnableEntityTypes[entityModelName], worldTransf);
        }
        else
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("Entity spawning failed - [%s] given name actor already exists!"), *entityName);
        }
    }
    PreviousSpawnRequest = InRequest;
    return newEntity;
}

bool ASimulationState::ServerCheckDeleteRequest(const FROSDeleteEntity_Request& InRequest)
{
    return (PreviousDeleteRequest.name != InRequest.name);
}

void ASimulationState::ServerDeleteEntity(const FROSDeleteEntity_Request& InRequest)
{
    if (ServerCheckDeleteRequest(InRequest))
    {
        AActor* Removed = Entities.FindAndRemoveChecked(InRequest.name);
        Removed->Destroy();
    }
    PreviousDeleteRequest = InRequest;
}
