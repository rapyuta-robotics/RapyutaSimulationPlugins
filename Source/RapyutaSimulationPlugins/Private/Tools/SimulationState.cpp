// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/SimulationState.h"

// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

// rclUE
#include "Srvs/ROS2Attach.h"
#include "Srvs/ROS2DeleteEntity.h"
#include "Srvs/ROS2GetEntityState.h"
#include "Srvs/ROS2SetEntityState.h"
#include "Srvs/ROS2SpawnEntities.h"
#include "Srvs/ROS2SpawnEntity.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRAssetUtils.h"
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"
#include "Net/UnrealNetwork.h"
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
        UE_LOG_WITH_INFO(LogRapyutaCore, Fatal, TEXT("[%s] should be called by server only"), *InFunctionName);
        return false;
    }
    return true;
}

void ASimulationState::RegisterSpawnableBPEntities(const TArray<FString>& InBPSpawnableClassNames)
{
    for (const auto& bpName : InBPSpawnableClassNames)
    {
        UClass* bpClass = URRAssetUtils::FindBlueprintClass(bpName);
        if (bpClass)
        {
            AddSpawnableEntityTypes({{bpName, bpClass}});
        }
    }
}

void ASimulationState::InitEntities()
{
#if RAPYUTA_SIM_VERBOSE
    TArray<AActor*> allActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), allActors);
    UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Found %d actors in the scene"), allActors.Num());
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

bool ASimulationState::ServerCheckSetEntityStateRequest(const FROSSetEntityStateReq& InRequest)
{
    if (PrevSetEntityStateRequest.State.Name == InRequest.State.Name &&
        PrevSetEntityStateRequest.State.ReferenceFrame == InRequest.State.ReferenceFrame &&
        PrevSetEntityStateRequest.State.Pose.Position == InRequest.State.Pose.Position &&
        PrevSetEntityStateRequest.State.Pose.Orientation == InRequest.State.Pose.Orientation)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void ASimulationState::ServerSetEntityState(const FROSSetEntityStateReq& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerSetEntityState")))
    {
        return;
    }

    if (ServerCheckSetEntityStateRequest(InRequest))
    {
        FTransform relativeTransf(InRequest.State.Pose.Orientation, InRequest.State.Pose.Position);
        relativeTransf = URRConversionUtils::TransformROSToUE(relativeTransf);
        FTransform worldTransf;
        URRGeneralUtils::GetWorldTransform(
            Entities.Contains(InRequest.State.ReferenceFrame) ? Entities[InRequest.State.ReferenceFrame] : nullptr,
            relativeTransf,
            worldTransf);
        Entities[InRequest.State.Name]->SetActorTransform(worldTransf);
    }

    PrevSetEntityStateRequest = InRequest;
}

bool ASimulationState::ServerCheckAttachRequest(const FROSAttachReq& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerCheckAttachRequest")))
    {
        return false;
    }
    return ((PrevAttachEntityRequest.Name1 != InRequest.Name1) || (PrevAttachEntityRequest.Name2 != InRequest.Name2));
}

void ASimulationState::ServerAttach(const FROSAttachReq& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerAttach")))
    {
        return;
    }

    // TODO: Add proper server check
    //if (ServerCheckAttachRequest(InRequest))

    AActor* entity1 = Entities[InRequest.Name1];
    AActor* entity2 = Entities[InRequest.Name2];

    if (entity2->IsRootComponentMovable())
    {
        if (!entity2->IsAttachedTo(entity1))
        {
            entity2->AttachToActor(entity1, FAttachmentTransformRules::KeepWorldTransform);

            // disable collision check with attached actor (Entity2) when entity1 moves
            for (auto component : entity1->GetComponents())
            {
                auto primitiveComp = Cast<UPrimitiveComponent>(component);
                if (primitiveComp)
                {
                    primitiveComp->IgnoreActorWhenMoving(entity2, true);
                }
            }
        }
        else
        {
            entity2->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);

            // enable collisions between the 2 actors when entity1 moves
            for (auto component : entity1->GetComponents())
            {
                auto primitiveComp = Cast<UPrimitiveComponent>(component);
                if (primitiveComp)
                {
                    primitiveComp->IgnoreActorWhenMoving(entity2, false);
                }
            }
        }
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Warning,
                         TEXT("entity2 to attach or detach %s has its Mobility not set as Movable. Please set Mobility of "
                              "entity2 as Movable for Attach service to work properly."),
                         *InRequest.Name2);
    }

    UE_LOG_WITH_INFO(
        LogRapyutaCore,
        Warning,
        TEXT("Entity %s and/or %s not exit or not under SimulationState Actor control. Please call AddEntity to make Actors "
             "under SimulationState control."),
        *InRequest.Name1,
        *InRequest.Name2);

    PrevAttachEntityRequest = InRequest;
}

bool ASimulationState::ServerCheckSpawnRequest(const FROSSpawnEntityReq& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerCheckSpawnRequest")))
    {
        return false;
    }

    if (PrevSpawnEntityRequest.Xml == InRequest.Xml && PrevSpawnEntityRequest.RobotNamespace == InRequest.RobotNamespace &&
        PrevSpawnEntityRequest.State.Name == InRequest.State.Name &&
        PrevSpawnEntityRequest.State.Pose.Position == InRequest.State.Pose.Position &&
        PrevSpawnEntityRequest.State.Pose.Orientation == InRequest.State.Pose.Orientation &&
        PrevSpawnEntityRequest.State.ReferenceFrame == InRequest.State.ReferenceFrame)
    {
        return false;
    }
    else
    {
        return true;
    }
}

AActor* ASimulationState::ServerSpawnEntity(const FROSSpawnEntityReq& InROSSpawnRequest,
                                            const TSubclassOf<AActor>& InEntityClass,
                                            const FTransform& InEntityTransform,
                                            const int32& InNetworkPlayerId)
{
    if (false == VerifyIsServerCall(TEXT("ServerSpawnEntity")))
    {
        return nullptr;
    }

    // SpawnActorDeferred to set parameters beforehand
    // Using AdjustIfPossibleButAlwaysSpawn, the actual entity's transform could be different from one specified in SpawnEntity,
    // thus we may need to inform ros side to get synchronized with it
    AActor* newEntity = GetWorld()->SpawnActorDeferred<AActor>(
        InEntityClass, InEntityTransform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn);
    if (newEntity == nullptr)
    {
        return nullptr;
    }

    // Wrap [InROSSpawnRequest] into a ROS 2 spawnable component
    UROS2Spawnable* spawnableComponent = NewObject<UROS2Spawnable>(newEntity, TEXT("ROS 2 Spawn Parameters"));
    spawnableComponent->RegisterComponent();
    spawnableComponent->SetNetworkPlayerId(InNetworkPlayerId);
    spawnableComponent->InitializeParameters(InROSSpawnRequest);

    newEntity->AddInstanceComponent(spawnableComponent);
    newEntity->Rename(*InROSSpawnRequest.State.Name);
#if WITH_EDITOR
    newEntity->SetActorLabel(*InROSSpawnRequest.State.Name);
#endif

    // Set/Configure [robot]'s [ROSSpawnParameters] as [spawnableComponent]

    // temporary solution to spawn
    // if there is a option to catch the spawn failure, we should avoid this.
    // newEntity->SpawnCollisionHandlingMethod = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    ARRBaseRobot* robot = Cast<ARRBaseRobot>(newEntity);
    if (robot)
    {
        robot->RobotUniqueName = InROSSpawnRequest.State.Name;

        // todo child actor component is not replicated.
        // https://forums.unrealengine.com/t/child-actor-component-never-replicated/23497/18
        robot->ROSSpawnParameters = spawnableComponent;
        robot->ServerRobot = robot;
    }
    else
    {
        newEntity->AddOwnedComponent(spawnableComponent);
    }

    // Add tags
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Log,
                     TEXT("[%s] tag from spawn request %s"),
                     *newEntity->GetName(),
                     *FString::Join(InROSSpawnRequest.Tags, TEXT(",")));
    for (const auto& tag : InROSSpawnRequest.Tags)
    {
        newEntity->Tags.Emplace(tag);
        spawnableComponent->AddTag(tag);
    }

    // Finish spawning Entity
    // Destroy seems not make newEntity=nullptr evevn if it failed.
    newEntity = UGameplayStatics::FinishSpawningActor(newEntity, InEntityTransform);

    // Add to [Entities]
    ServerAddEntity(newEntity);

    return newEntity;
}

AActor* ASimulationState::ServerSpawnEntity(const FROSSpawnEntityReq& InRequest, const int32 InNetworkPlayerId)
{
    if (false == VerifyIsServerCall(TEXT("ServerSpawnEntity")))
    {
        return nullptr;
    }

    AActor* newEntity = nullptr;
    if (ServerCheckSpawnRequest(InRequest))
    {
        const FString& entityModelName = InRequest.Xml;
        const FString& entityName = InRequest.State.Name;
        verify(false == entityName.IsEmpty());
        if (nullptr == URRGeneralUtils::FindActorByName<AActor>(GetWorld(), entityName))
        {
            // Calculate to-be-spawned entity's [world transf]
            FTransform relativeTransf =
                URRConversionUtils::TransformROSToUE(FTransform(InRequest.State.Pose.Orientation, InRequest.State.Pose.Position));
            const FString& referenceFrame = InRequest.State.ReferenceFrame;
            FTransform worldTransf;
            URRGeneralUtils::GetWorldTransform(Entities.FindRef(referenceFrame), relativeTransf, worldTransf);

            // Spawn entity
            newEntity = ServerSpawnEntity(InRequest, SpawnableEntityTypes[entityModelName], worldTransf, InNetworkPlayerId);
            if (newEntity)
            {
                UE_LOG_WITH_INFO(LogRapyutaCore,
                                 Warning,
                                 TEXT("Spawned Entity of model [%s] as [%s] to world pose: %s - ReferenceFrame: %s"),
                                 *entityModelName,
                                 *entityName,
                                 *worldTransf.ToString(),
                                 *referenceFrame);
            }
            else
            {
                // todo: need pass response to SimulationStateClient
                // response.bSuccess = false;
                // response.StatusMessage =
                //     FString::Printf(TEXT("[%s] Failed to spawn entity named %s, probably out of collision!"), *GetName(),
                //     *entityName);
                UE_LOG_WITH_INFO(LogRapyutaCore,
                                 Error,
                                 TEXT("[ASimulationState] Failed to spawn entity named %s, probably out of collision!"),
                                 *entityName);
            }
        }
        else
        {
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Error, TEXT("Entity spawning failed - [%s] given name actor already exists!"), *entityName);
        }
    }
    PrevSpawnEntityRequest = InRequest;
    return newEntity;
}

bool ASimulationState::ServerCheckDeleteRequest(const FROSDeleteEntityReq& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerCheckDeleteRequest")))
    {
        return false;
    }
    return (PrevDeleteEntityRequest.Name != InRequest.Name);
}

void ASimulationState::ServerDeleteEntity(const FROSDeleteEntityReq& InRequest)
{
    if (false == VerifyIsServerCall(TEXT("ServerDeleteEntity")))
    {
        return;
    }

    if (ServerCheckDeleteRequest(InRequest))
    {
        AActor* Removed = Entities.FindAndRemoveChecked(InRequest.Name);
        Removed->Destroy();
    }
    PrevDeleteEntityRequest = InRequest;
}
