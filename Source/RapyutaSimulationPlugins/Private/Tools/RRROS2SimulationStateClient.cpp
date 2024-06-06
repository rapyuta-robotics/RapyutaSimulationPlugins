// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2SimulationStateClient.h"

// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "Net/UnrealNetwork.h"
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
#include "Core/RRConversionUtils.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRROS2GameMode.h"
#include "Core/RRUObjectUtils.h"
#include "Tools/ROS2Spawnable.h"
#include "Tools/SimulationState.h"

void URRROS2SimulationStateClient::OnComponentCreated()
{
    Super::OnComponentCreated();
    SetIsReplicated(true);
}

void URRROS2SimulationStateClient::Init(UROS2NodeComponent* InROS2Node)
{
    ROS2Node = InROS2Node;

    ROS2_CREATE_SERVICE_SERVER(ROS2Node,
                               this,
                               TEXT("GetEntityState"),
                               UROS2GetEntityStateSrv::StaticClass(),
                               &URRROS2SimulationStateClient::GetEntityStateSrv);
    ROS2_CREATE_SERVICE_SERVER(ROS2Node,
                               this,
                               TEXT("SetEntityState"),
                               UROS2SetEntityStateSrv::StaticClass(),
                               &URRROS2SimulationStateClient::SetEntityStateSrv);
    ROS2_CREATE_SERVICE_SERVER(
        ROS2Node, this, TEXT("Attach"), UROS2AttachSrv::StaticClass(), &URRROS2SimulationStateClient::AttachSrv);
    ROS2_CREATE_SERVICE_SERVER(
        ROS2Node, this, TEXT("SpawnEntity"), UROS2SpawnEntitySrv::StaticClass(), &URRROS2SimulationStateClient::SpawnEntitySrv);
    ROS2_CREATE_SERVICE_SERVER(ROS2Node,
                               this,
                               TEXT("SpawnEntities"),
                               UROS2SpawnEntitiesSrv::StaticClass(),
                               &URRROS2SimulationStateClient::SpawnEntitiesSrv);
    ROS2_CREATE_SERVICE_SERVER(
        ROS2Node, this, TEXT("DeleteEntity"), UROS2DeleteEntitySrv::StaticClass(), &URRROS2SimulationStateClient::DeleteEntitySrv);
}

void URRROS2SimulationStateClient::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URRROS2SimulationStateClient, ROS2Node);
    DOREPLIFETIME(URRROS2SimulationStateClient, ServerSimState);
    DOREPLIFETIME(URRROS2SimulationStateClient, NetworkPlayerId);
}

template<typename T>
bool URRROS2SimulationStateClient::CheckEntity(TMap<FString, T>& InEntities, const FString& InEntityName, const bool bAllowEmpty)
{
    bool result = false;
    if (InEntities.Contains(InEntityName))
    {
        if (IsValid(InEntities[InEntityName]))
        {
            result = true;
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(
                LogRapyutaCore, Warning, TEXT("Entity named [%s] gets invalid -> removed from Entities"), *InEntityName);
            InEntities.Remove(InEntityName);
        }
    }
    else if (bAllowEmpty && InEntityName.IsEmpty())
    {
        result = true;
    }
    else
    {
        FString temp = TEXT("");
        for (auto& Elem : InEntities)
        {
            temp += Elem.Key + TEXT(", ");
        }
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore,
                               Warning,
                               TEXT("Entity named [%s] is not under SimulationState control. Please register it to "
                                    "SimulationState! \n Entities in control are: %s"),

                               *InEntityName,
                               *temp);
    }
    return result;
}

// https://isocpp.org/wiki/faq/templates#templates-defn-vs-decl
template bool URRROS2SimulationStateClient::CheckEntity<AActor*>(TMap<FString, AActor*>& InEntities,
                                                                 const FString& InEntityName,
                                                                 const bool bAllowEmpty);

template bool URRROS2SimulationStateClient::CheckEntity<TSubclassOf<AActor>>(TMap<FString, TSubclassOf<AActor>>& InEntities,
                                                                             const FString& InEntityName,
                                                                             const bool bAllowEmpty);

bool URRROS2SimulationStateClient::CheckEntity(const FString& InEntityName, const bool bAllowEmpty)
{
    return CheckEntity<AActor*>(ServerSimState->Entities, InEntityName, bAllowEmpty);
}

#if WITH_EDITOR
bool URRROS2SimulationStateClient::CheckEntityWithDisplayName(const FString& InEntityName, const bool bAllowEmpty)
{
    return CheckEntity<AActor*>(ServerSimState->EntitiesWithDisplayName, InEntityName, bAllowEmpty);
}
#endif

bool URRROS2SimulationStateClient::CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty)
{
    return CheckEntity<TSubclassOf<AActor>>(ServerSimState->SpawnableEntityTypes, InEntityName, bAllowEmpty);
}

void URRROS2SimulationStateClient::GetEntityStateSrv(UROS2GenericSrv* InService)
{
    UROS2GetEntityStateSrv* GetEntityStateService = Cast<UROS2GetEntityStateSrv>(InService);

    FROSGetEntityStateReq request;
    GetEntityStateService->GetRequest(request);

    FROSGetEntityStateRes response;
    response.State.Name = request.Name;
    response.State.ReferenceFrame = request.ReferenceFrame;

    AActor* TargetActor = ServerSimState->Entities.FindRef(request.Name);
    AActor* RefActor = ServerSimState->Entities.FindRef(request.ReferenceFrame);
    response.bSuccess = CheckEntity(request.Name, false) && CheckEntity(request.ReferenceFrame, true);
#if WITH_EDITOR
    if (TargetActor == nullptr)
    {
        TargetActor = ServerSimState->EntitiesWithDisplayName.FindRef(request.Name);
    }
    if (RefActor == nullptr)
    {
        RefActor = ServerSimState->EntitiesWithDisplayName.FindRef(request.ReferenceFrame);
    }
    response.bSuccess |=
        CheckEntityWithDisplayName(request.Name, false) && CheckEntityWithDisplayName(request.ReferenceFrame, true);
#endif
    if (response.bSuccess)
    {
        FTransform worldTransf = TargetActor->GetTransform();
        FTransform relativeTransf = URRGeneralUtils::GetRelativeTransform(RefActor, worldTransf);
        relativeTransf = URRConversionUtils::TransformUEToROS(relativeTransf);

        response.State.Pose.Position = relativeTransf.GetTranslation();
        response.State.Pose.Orientation = relativeTransf.GetRotation();

        response.State.Twist.Linear = FVector::ZeroVector;
        response.State.Twist.Angular = FVector::ZeroVector;
    }

    GetEntityStateService->SetResponse(response);
}

void URRROS2SimulationStateClient::SetEntityStateSrv(UROS2GenericSrv* InService)
{
    UROS2SetEntityStateSrv* setEntityStateService = Cast<UROS2SetEntityStateSrv>(InService);

    FROSSetEntityStateReq request;
    setEntityStateService->GetRequest(request);

    FROSSetEntityStateRes response;
    response.bSuccess = CheckEntity(request.State.Name, false) && CheckEntity(request.State.ReferenceFrame, true);
#if WITH_EDITOR
    response.bSuccess |=
        CheckEntityWithDisplayName(request.State.Name, false) && CheckEntityWithDisplayName(request.State.ReferenceFrame, true);
#endif
    if (response.bSuccess)
    {
        // RPC to Server
        ServerSetEntityState(request);
    }

    setEntityStateService->SetResponse(response);
}

void URRROS2SimulationStateClient::ServerSetEntityState_Implementation(const FROSSetEntityStateReq& InRequest)
{
    ServerSimState->ServerSetEntityState(InRequest);
}

void URRROS2SimulationStateClient::AttachSrv(UROS2GenericSrv* InService)
{
    UROS2AttachSrv* attachService = Cast<UROS2AttachSrv>(InService);

    FROSAttachReq request;
    attachService->GetRequest(request);

    FROSAttachRes response;
    response.bSuccess = CheckEntity(request.Name1, false) && CheckEntity(request.Name2, false);
#if WITH_EDITOR
    response.bSuccess |= CheckEntityWithDisplayName(request.Name1, false) && CheckEntityWithDisplayName(request.Name2, false);
#endif
    if (response.bSuccess)
    {
        // RPC to server
        ServerAttach(request);
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(
            LogRapyutaCore,
            Warning,
            TEXT("Entity %s and/or %s not exit or not under SimulationState Actor control. Please call ServerAddEntity to make "
                 "Actors under SimulationState control."),
            *request.Name1,
            *request.Name2);
    }

    attachService->SetResponse(response);
}

void URRROS2SimulationStateClient::ServerAttach_Implementation(const FROSAttachReq& InRequest)
{
    ServerSimState->ServerAttach(InRequest);
}

FROSSpawnEntityRes URRROS2SimulationStateClient::SpawnEntityImpl(FROSSpawnEntityReq& InRequest)
{
    FROSSpawnEntityRes response;
    response.bSuccess = CheckSpawnableEntity(InRequest.Xml, false) && CheckEntity(InRequest.State.ReferenceFrame, true);
#if WITH_EDITOR
    response.bSuccess |=
        CheckSpawnableEntity(InRequest.Xml, false) && CheckEntityWithDisplayName(InRequest.State.ReferenceFrame, true);
#endif
    if (response.bSuccess)
    {
        const FString& entityModelName = InRequest.Xml;
        const FString& entityName = InRequest.State.Name;
        if (entityName.IsEmpty())
        {
            response.bSuccess = false;
            response.StatusMessage = FString::Printf(
                TEXT("[%s] Failed to spawn entity of model [%s]. Entity Name is empty"), *GetName(), *entityModelName);
        }
        else if (nullptr == URRGeneralUtils::FindActorByName<AActor>(GetWorld(), entityName))
        {
            // RPC to Server's Spawn entity
            ServerSpawnEntity(InRequest);

            // RPC is not blocking and can't get actor even if it is spawned.
            // todo: handle failed to spawn with collision and etc.

            // AActor* newEntity = URRGeneralUtils::FindActorByName<AActor>(GetWorld(), entityName);
            // if (nullptr == newEntity)
            // {
            //     response.bSuccess = false;
            //     response.StatusMessage =
            //         FString::Printf(TEXT("[%s] Failed to spawn entity named %s, probably out of collision!"), *GetName(),
            //         *entityName);
            // }
            // else
            // {
            response.bSuccess = true;
            // response.StatusMessage = FString::Printf(TEXT("Spawning Entity of model [%s] as [%s]"), *entityModelName,
            // *entityName);
            // }
        }
        else
        {
            response.bSuccess = false;
            response.StatusMessage = FString::Printf(
                TEXT("[%s] Failed to spawn entity named %s,  given name actor already exists!"), *GetName(), *entityName);
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("%s"), *response.StatusMessage);
        }
    }
    return response;
}

void URRROS2SimulationStateClient::SpawnEntitySrv(UROS2GenericSrv* InService)
{
    UROS2SpawnEntitySrv* SpawnEntityService = Cast<UROS2SpawnEntitySrv>(InService);

    FROSSpawnEntityReq request;
    SpawnEntityService->GetRequest(request);
    SpawnEntityService->SetResponse(SpawnEntityImpl(request));
}

void URRROS2SimulationStateClient::SpawnEntitiesSrv(UROS2GenericSrv* InService)
{
    UROS2SpawnEntitiesSrv* spawnEntitiesService = Cast<UROS2SpawnEntitiesSrv>(InService);
    FROSSpawnEntitiesReq entityListRequest;
    spawnEntitiesService->GetRequest(entityListRequest);

    int32 numEntitySpawned = 0;
    FString statusMessage;
    for (uint32 i = 0; i < entityListRequest.State.Num(); ++i)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Log,
                         TEXT("Spawning Entity : %s (name: %s)"),
                         *entityListRequest.Type[i],
                         *entityListRequest.State[i].Name);
        FROSSpawnEntityReq entityRequest;
        entityRequest.Xml = entityListRequest.Type[i];
        entityRequest.RobotNamespace = EMPTY_STR;
        entityRequest.State = entityListRequest.State[i];
        entityRequest.Tags = entityListRequest.Tags;

        FROSSpawnEntityRes res = SpawnEntityImpl(entityRequest);
        if (res.bSuccess)
        {
            numEntitySpawned++;
        }
        statusMessage.Append(FString::Printf(TEXT("%s:%s, "), *entityRequest.State.Name, *res.StatusMessage));
    }

    FROSSpawnEntitiesRes entityListResponse;
    entityListResponse.bSuccess = (numEntitySpawned == entityListRequest.State.Num());
    entityListResponse.StatusMessage = statusMessage;
    spawnEntitiesService->SetResponse(entityListResponse);
}

void URRROS2SimulationStateClient::ServerSpawnEntity_Implementation(const FROSSpawnEntityReq& InRequest)
{
    ServerSimState->ServerSpawnEntity(InRequest, NetworkPlayerId);
}

// Currently this code doesnt seem to trigger the ROS 2 Service Response... keeping this in since if
// that can be figured out, we can have better verification of spawned actors
// Code to do checking of if Entity is spawned using 2 timers, one for timeout and one for triggering this every x s
#if 0
void URRROS2SimulationStateClient::SpawnEntityCheck()
{
    if (IsNetMode(NM_Client))
    {
        FROSSpawnEntityReq request;
        SpawnEntityService->GetRequest(request);

        AActor* matchingEntity;
        for (auto& entity : SimulationState->EntityList)
        {
            if (entity)
            {
                UROS2Spawnable* rosSpawnParameters = Entity->FindComponentByClass<UROS2Spawnable>();
                if (rosSpawnParameters)
                {
                    if (rosSpawnParameters->GetName() == request.State.Name)
                    {
                        matchingEntity = entity;
                        break;
                    }
                }
            }
        }
        if (matchingEntity)
        {
            SpawnResponse.bSuccess = true;
            SpawnResponse.StatusMessage = FString::Printf(TEXT("Newly spawned Entity: %s"), *request.State.Name);
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("%s"), *SpawnResponse.StatusMessage);
            SpawnEntityService->SetResponse(SpawnResponse);
            GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
        }
        else
        {
            SpawnResponse.bSuccess = false;
            SpawnResponse.StatusMessage = FString::Printf(TEXT("Entity spawning failed for actor [%s]"), *request.State.Name);
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("%s"), *SpawnResponse.StatusMessage);
        }
    }
}

void URRROS2SimulationStateClient::SpawnEntityResponse()
{
    SpawnEntityService->SetResponse(SpawnResponse);
    GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
}
#endif

void URRROS2SimulationStateClient::DeleteEntitySrv(UROS2GenericSrv* InService)
{
    UROS2DeleteEntitySrv* deleteEntityService = Cast<UROS2DeleteEntitySrv>(InService);

    FROSDeleteEntityReq request;
    deleteEntityService->GetRequest(request);

    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Deleting %s"), *request.Name);

    FROSDeleteEntityRes response;
    response.bSuccess = false;
    if (ServerSimState->Entities.Contains(request.Name))
    {
        // RPC to server
        ServerDeleteEntity(request);
        response.bSuccess = true;
        response.StatusMessage = FString::Printf(TEXT("[%s] Deleted Entity named %s"), *GetName(), *request.Name);
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("%s"), *response.StatusMessage);
    }
    else
    {
        response.StatusMessage = FString::Printf(
            TEXT("[%s] Failed to delete entity named %s. %s do not exist."), *GetName(), *request.Name, *request.Name);
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("%s"), *response.StatusMessage);
    }

    deleteEntityService->SetResponse(response);
}

void URRROS2SimulationStateClient::ServerDeleteEntity_Implementation(const FROSDeleteEntityReq& InRequest)
{
    ServerSimState->ServerDeleteEntity(InRequest);
}

void URRROS2SimulationStateClient::ServerAddEntity_Implementation(AActor* InEntity)
{
    ServerSimState->ServerAddEntity(InEntity);
}
