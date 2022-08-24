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
#include "Robots/RRBaseRobot.h"
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

bool ASimulationState::VerifyIsServerCall(const FString& InFunctionName)
{
    if (IsNetMode(NM_Client))
    {
        UE_LOG(LogRapyutaCore, Fatal, TEXT("[%s] should be called by server only"), *InFunctionName);
        return false;
    }
    return true;
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
        ServerAddEntity(actor);
    }

    // NOTE: [SpawnableEntityInfoList] is a TArray<> thus replicatable, which is not supported for [SpawnableEntities] as a TMap
    GetWorld()->GetTimerManager().SetTimer(
        FetchEntityListTimerHandle, this, &ASimulationState::GetSpawnableEntityInfoList, 1.0f, true);
}

void ASimulationState::ServerAddEntity(AActor* InEntity)
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
void ASimulationState::OnRep_EntityList()
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
            AddTaggedEntity(entity, tag);
        }

        UROS2Spawnable* EntitySpawnParam = entity->FindComponentByClass<UROS2Spawnable>();
        if (EntitySpawnParam)
        {
            entity->Rename(*EntitySpawnParam->GetName());
            for (const auto& tag : EntitySpawnParam->ActorTags)
            {
                AddTaggedEntity(entity, FName(tag));
            }
        }
    }
}

void ASimulationState::OnRep_SpawnableEntityInfoList()
{
    for (const auto& entityInfo : SpawnableEntityInfoList)
    {
        SpawnableEntityTypes.Emplace(entityInfo.EntityTypeName, entityInfo.EntityClass);
    }
}

void ASimulationState::AddTaggedEntity(AActor* Entity, const FName& InTag)
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
        GetWorld()->GetTimerManager().ClearTimer(FetchEntityListTimerHandle);
    }
}

bool ASimulationState::ServerCheckSetEntityStateRequest(const FROSSetEntityState_Request& InRequest)
{
    if (PrevSetEntityStateRequest.state_name == InRequest.state_name &&
        PrevSetEntityStateRequest.state_reference_frame == InRequest.state_reference_frame &&
        PrevSetEntityStateRequest.state_pose_position_x == InRequest.state_pose_position_x &&
        PrevSetEntityStateRequest.state_pose_position_y == InRequest.state_pose_position_y &&
        PrevSetEntityStateRequest.state_pose_position_z == InRequest.state_pose_position_z &&
        PrevSetEntityStateRequest.state_pose_orientation == InRequest.state_pose_orientation)
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
    if (false == VerifyIsServerCall(TEXT("ServerSetEntityState")))
    {
        return;
    }

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

    PrevSetEntityStateRequest = InRequest;
}

bool ASimulationState::ServerCheckAttachRequest(const FROSAttach_Request& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerCheckAttachRequest")))
    {
        return false;
    }
    return ((PrevAttachEntityRequest.name1 != InRequest.name1) || (PrevAttachEntityRequest.name2 != InRequest.name2));
}

void ASimulationState::ServerAttach(const FROSAttach_Request& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerAttach")))
    {
        return;
    }

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

    PrevAttachEntityRequest = InRequest;
}

bool ASimulationState::ServerCheckSpawnRequest(const FROSSpawnEntityRequest& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerCheckSpawnRequest")))
    {
        return false;
    }

    if (PrevSpawnEntityRequest.Xml == InRequest.Xml && PrevSpawnEntityRequest.RobotNamespace == InRequest.RobotNamespace &&
        PrevSpawnEntityRequest.StateName == InRequest.StateName &&
        PrevSpawnEntityRequest.StatePosePositionX == InRequest.StatePosePositionX &&
        PrevSpawnEntityRequest.StatePosePositionY == InRequest.StatePosePositionY &&
        PrevSpawnEntityRequest.StatePosePositionZ == InRequest.StatePosePositionZ &&
        PrevSpawnEntityRequest.StatePoseOrientation == InRequest.StatePoseOrientation &&
        PrevSpawnEntityRequest.StateReferenceFrame == InRequest.StateReferenceFrame)
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
                                            const FTransform& InEntityTransform,
                                            const int32& InNetworkPlayerId)
{
    if (false == VerifyIsServerCall(TEXT("ServerSpawnEntity")))
    {
        return nullptr;
    }

    // SpawnActorDeferred to set parameters beforehand
    AActor* newEntity = GetWorld()->SpawnActorDeferred<AActor>(InEntityClass, InEntityTransform);
    if (newEntity == nullptr)
    {
        return nullptr;
    }
    
    // Wrap [InROSSpawnRequest] into a ROS2 spawnable component
    UROS2Spawnable* spawnableComponent = NewObject<UROS2Spawnable>(newEntity, TEXT("ROS2 Spawn Parameters"));
    spawnableComponent->SetIsReplicated(true);
    spawnableComponent->RegisterComponent();
    spawnableComponent->SetNetworkPlayerId(InNetworkPlayerId);
    spawnableComponent->InitializeParameters(InROSSpawnRequest);

    newEntity->AddInstanceComponent(spawnableComponent);
    newEntity->Rename(*InROSSpawnRequest.StateName);
#if WITH_EDITOR
    newEntity->SetActorLabel(*InROSSpawnRequest.StateName);
#endif
    ARRBaseRobot* robot = Cast<ARRBaseRobot>(newEntity);
    if (robot)
    {
        robot->RobotUniqueName = InROSSpawnRequest.StateName;

        // todo child actor component is not replicated.
        // https://forums.unrealengine.com/t/child-actor-component-never-replicated/23497/18
        robot->ROSSpawnParameters = spawnableComponent;
        robot->ServerRobot = robot;
    }

    // Add tags
    UE_LOG(LogRapyutaCore,
           Log,
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
    ServerAddEntity(newEntity);
    return newEntity;
}

AActor* ASimulationState::ServerSpawnEntity(const FROSSpawnEntityRequest& InRequest, const int32 InNetworkPlayerId)
{
    if (false == VerifyIsServerCall(TEXT("ServerSpawnEntity")))
    {
        return nullptr;
    }

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
            newEntity = ServerSpawnEntity(InRequest, SpawnableEntityTypes[entityModelName], worldTransf, InNetworkPlayerId);
            if (nullptr == newEntity)
            {
                // todo: need pass response to SimulationStateClient
                // response.bSuccess = false;
                // response.StatusMessage =
                //     FString::Printf(TEXT("[%s] Failed to spawn entity named %s, probably out collision!"), *GetName(),
                //     *entityName);
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("[ASimulationState] Failed to spawn entity named %s, probably out collision!"),
                       *entityName);
            }

        }
        else
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("Entity spawning failed - [%s] given name actor already exists!"), *entityName);
        }
    }
    PrevSpawnEntityRequest = InRequest;
    return newEntity;
}

bool ASimulationState::ServerCheckDeleteRequest(const FROSDeleteEntity_Request& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerCheckDeleteRequest")))
    {
        return false;
    }
    return (PrevDeleteEntityRequest.name != InRequest.name);
}

void ASimulationState::ServerDeleteEntity(const FROSDeleteEntity_Request& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerDeleteEntity")))
    {
        return;
    }

    if (ServerCheckDeleteRequest(InRequest))
    {
        AActor* Removed = Entities.FindAndRemoveChecked(InRequest.name);
        Removed->Destroy();
    }
    PrevDeleteEntityRequest = InRequest;
}
