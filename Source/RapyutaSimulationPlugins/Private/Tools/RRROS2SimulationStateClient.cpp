// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2SimulationStateClient.h"

// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "Net/UnrealNetwork.h"
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

void URRROS2SimulationStateClient::Init(AROS2Node* InROS2Node)
{
    ROS2Node = InROS2Node;

    // register delegates to node
    FServiceCallback GetEntityStateSrvCallback;
    FServiceCallback SetEntityStateSrvCallback;
    FServiceCallback AttachSrvCallback;
    FServiceCallback SpawnEntitySrvCallback;
    FServiceCallback SpawnEntitiesSrvCallback;
    FServiceCallback DeleteEntitySrvCallback;
    GetEntityStateSrvCallback.BindDynamic(this, &URRROS2SimulationStateClient::GetEntityStateSrv);
    SetEntityStateSrvCallback.BindDynamic(this, &URRROS2SimulationStateClient::SetEntityStateSrv);
    AttachSrvCallback.BindDynamic(this, &URRROS2SimulationStateClient::AttachSrv);
    SpawnEntitySrvCallback.BindDynamic(this, &URRROS2SimulationStateClient::SpawnEntitySrv);
    SpawnEntitiesSrvCallback.BindDynamic(this, &URRROS2SimulationStateClient::SpawnEntitiesSrv);
    DeleteEntitySrvCallback.BindDynamic(this, &URRROS2SimulationStateClient::DeleteEntitySrv);
    InROS2Node->AddServiceServer(TEXT("GetEntityState"), UROS2GetEntityStateSrv::StaticClass(), GetEntityStateSrvCallback);
    InROS2Node->AddServiceServer(TEXT("SetEntityState"), UROS2SetEntityStateSrv::StaticClass(), SetEntityStateSrvCallback);
    InROS2Node->AddServiceServer(TEXT("Attach"), UROS2AttachSrv::StaticClass(), AttachSrvCallback);
    InROS2Node->AddServiceServer(TEXT("SpawnEntity"), UROS2SpawnEntitySrv::StaticClass(), SpawnEntitySrvCallback);
    InROS2Node->AddServiceServer(TEXT("SpawnEntities"), UROS2SpawnEntitiesSrv::StaticClass(), SpawnEntitiesSrvCallback);
    InROS2Node->AddServiceServer(TEXT("DeleteEntity"), UROS2DeleteEntitySrv::StaticClass(), DeleteEntitySrvCallback);
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
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("[%s] Entity named [%s] gets invalid -> removed from Entities"),
                   *GetName(),
                   *InEntityName);
            InEntities.Remove(InEntityName);
        }
    }
    else if (bAllowEmpty && InEntityName.IsEmpty())
    {
        result = true;
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("[%s] Entity named [%s] is not under SimulationState control. Please register it to SimulationState!"),
               *GetName(),
               *InEntityName);
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

bool URRROS2SimulationStateClient::CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty)
{
    return CheckEntity<TSubclassOf<AActor>>(ServerSimState->SpawnableEntityTypes, InEntityName, bAllowEmpty);
}

void URRROS2SimulationStateClient::GetEntityStateSrv(UROS2GenericSrv* InService)
{
    UROS2GetEntityStateSrv* GetEntityStateService = Cast<UROS2GetEntityStateSrv>(InService);

    FROSGetEntityStateRequest request;
    GetEntityStateService->GetRequest(request);

    FROSGetEntityStateResponse response;
    response.StateName = request.Name;
    response.bSuccess = CheckEntity(request.Name, false) && CheckEntity(request.ReferenceFrame, true);

    if (response.bSuccess)
    {
        FTransform relativeTransf;
        FTransform worldTransf = ServerSimState->Entities[request.Name]->GetTransform();
        URRGeneralUtils::GetRelativeTransform(
            request.ReferenceFrame, ServerSimState->Entities.FindRef(request.ReferenceFrame), worldTransf, relativeTransf);
        relativeTransf = URRConversionUtils::TransformUEToROS(relativeTransf);

        response.StatePosePosition.X = relativeTransf.GetTranslation().X;
        response.StatePosePosition.Y = relativeTransf.GetTranslation().Y;
        response.StatePosePosition.Z = relativeTransf.GetTranslation().Z;
        response.StatePoseOrientation = relativeTransf.GetRotation();

        response.StateTwistLinear = FVector::ZeroVector;
        response.StateTwistAngular = FVector::ZeroVector;
    }

    GetEntityStateService->SetResponse(response);
}

void URRROS2SimulationStateClient::SetEntityStateSrv(UROS2GenericSrv* InService)
{
    UROS2SetEntityStateSrv* setEntityStateService = Cast<UROS2SetEntityStateSrv>(InService);

    FROSSetEntityStateRequest request;
    setEntityStateService->GetRequest(request);

    FROSSetEntityStateResponse response;
    response.bSuccess = CheckEntity(request.StateName, false) && CheckEntity(request.StateReferenceFrame, true);

    if (response.bSuccess)
    {
        // RPC to Server
        ServerSetEntityState(request);
    }

    setEntityStateService->SetResponse(response);
}

void URRROS2SimulationStateClient::ServerSetEntityState_Implementation(const FROSSetEntityStateRequest& InRequest)
{
    ServerSimState->ServerSetEntityState(InRequest);
}

void URRROS2SimulationStateClient::AttachSrv(UROS2GenericSrv* InService)
{
    UROS2AttachSrv* attachService = Cast<UROS2AttachSrv>(InService);

    FROSAttachRequest request;
    attachService->GetRequest(request);

    FROSAttachResponse response;
    response.bSuccess = CheckEntity(request.Name1, false) && CheckEntity(request.Name2, false);
    if (response.bSuccess)
    {
        // RPC to server
        ServerAttach(request);
    }
    else
    {
        UE_LOG(
            LogRapyutaCore,
            Warning,
            TEXT(
                "[%s] Entity %s and/or %s not exit or not under SimulationState Actor control. Please call ServerAddEntity to make "
                "Actors "
                "under SimulationState control."),
            *GetName(),
            *request.Name1,
            *request.Name2);
    }

    attachService->SetResponse(response);
}

void URRROS2SimulationStateClient::ServerAttach_Implementation(const FROSAttachRequest& InRequest)
{
    ServerSimState->ServerAttach(InRequest);
}

void URRROS2SimulationStateClient::SpawnEntitySrv(UROS2GenericSrv* InService)
{
    UROS2SpawnEntitySrv* SpawnEntityService = Cast<UROS2SpawnEntitySrv>(InService);

    FROSSpawnEntityRequest request;
    SpawnEntityService->GetRequest(request);

    FROSSpawnEntityResponse response;
    response.bSuccess = CheckSpawnableEntity(request.Xml, false) && CheckEntity(request.StateReferenceFrame, true);
    if (response.bSuccess)
    {
        const FString& entityModelName = request.Xml;
        const FString& entityName = request.StateName;
        verify(false == entityName.IsEmpty());
        if (nullptr == URRUObjectUtils::FindActorByName<AActor>(GetWorld(), entityName))
        {
            // RPC to Server's Spawn entity
            ServerSpawnEntity(request);

            // RPC is not blocking and can't get actor even if it is spawned.
            // todo: handle failed to spawn with collision and etc.

            // AActor* newEntity = URRUObjectUtils::FindActorByName<AActor>(GetWorld(), entityName);
            // if (nullptr == newEntity)
            // {
            //     response.bSuccess = false;
            //     response.StatusMessage =
            //         FString::Printf(TEXT("[%s] Failed to spawn entity named %s, probably out collision!"), *GetName(),
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
            UE_LOG(LogRapyutaCore, Error, TEXT("%s"), *response.StatusMessage);
        }
    }
    SpawnEntityService->SetResponse(response);
}

void URRROS2SimulationStateClient::SpawnEntitiesSrv(UROS2GenericSrv* InService)
{
    UROS2SpawnEntitiesSrv* spawnEntitiesService = Cast<UROS2SpawnEntitiesSrv>(InService);
    FROSSpawnEntitiesRequest entityListRequest;
    spawnEntitiesService->GetRequest(entityListRequest);

    int32 numEntitySpawned = 0;
    FString statusMessage;
    for (uint32 i = 0; i < entityListRequest.StateName.Num(); ++i)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("Spawning Entity : %s (name: %s)"),
               *entityListRequest.Type[i],
               *entityListRequest.StateName[i]);
        FROSSpawnEntityRequest entityRequest;
        entityRequest.Xml = entityListRequest.Type[i];
        entityRequest.RobotNamespace = EMPTY_STR;
        entityRequest.StateName = entityListRequest.StateName[i];
        entityRequest.StatePosePosition.X = entityListRequest.StatePosePosition[i].X;
        entityRequest.StatePosePosition.Y = entityListRequest.StatePosePosition[i].Y;
        entityRequest.StatePosePosition.Z = entityListRequest.StatePosePosition[i].Z;
        entityRequest.StatePoseOrientation = entityListRequest.StatePoseOrientation[i];
        entityRequest.StateTwistLinear = entityListRequest.StateTwistLinear[i];
        entityRequest.StateTwistAngular = entityListRequest.StateTwistAngular[i];
        entityRequest.StateReferenceFrame = entityListRequest.StateReferenceFrame[i];
        entityRequest.Tags.Add(entityListRequest.Tags[i]);

        if (CheckSpawnableEntity(entityRequest.Xml, false) && CheckEntity(entityRequest.StateReferenceFrame, true))
        {
            if (nullptr == URRUObjectUtils::FindActorByName<AActor>(GetWorld(), entityRequest.StateName))
            {
                // RPC call to the server
                ServerSpawnEntity(entityRequest);
            }
            else
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("[%s] Failed to spawn entity named %s,  given name actor already exists!"),
                       *GetName(),
                       *entityRequest.StateName);
            }
        }

        statusMessage.Append(FString::Printf(TEXT("%s,"), *entityRequest.StateName));
    }

    FROSSpawnEntitiesResponse entityListResponse;
    entityListResponse.bSuccess = (numEntitySpawned == entityListRequest.StateName.Num());
    entityListResponse.StatusMessage = entityListResponse.bSuccess
                                         ? FString::Printf(TEXT("Newly spawned entities: [%s]"), *statusMessage)
                                         : FString::Printf(TEXT("Failed to be spawned entities: [%s]"), *statusMessage);

    spawnEntitiesService->SetResponse(entityListResponse);
}

void URRROS2SimulationStateClient::ServerSpawnEntity_Implementation(const FROSSpawnEntityRequest& InRequest)
{
    ServerSimState->ServerSpawnEntity(InRequest, NetworkPlayerId);
}

// Currently this code doesnt seem to trigger the ROS2 Service Response... keeping this in since if
// that can be figured out, we can have better verification of spawned actors
// Code to do checking of if Entity is spawned using 2 timers, one for timeout and one for triggering this every x s
#if 0
void URRROS2SimulationStateClient::SpawnEntityCheck()
{
    if (IsNetMode(NM_Client))
    {
        FROSSpawnEntityRequest request;
        SpawnEntityService->GetRequest(request);

        AActor* matchingEntity;
        for (auto& entity : SimulationState->EntityList)
        {
            if (entity)
            {
                UROS2Spawnable* rosSpawnParameters = Entity->FindComponentByClass<UROS2Spawnable>();
                if (rosSpawnParameters)
                {
                    if (rosSpawnParameters->GetName() == request.StateName)
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
            SpawnResponse.StatusMessage = FString::Printf(TEXT("Newly spawned Entity: %s"), *request.StateName);
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *SpawnResponse.StatusMessage);
            SpawnEntityService->SetResponse(SpawnResponse);
            GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
        }
        else
        {
            SpawnResponse.bSuccess = false;
            SpawnResponse.StatusMessage = FString::Printf(TEXT("Entity spawning failed for actor [%s]"), *request.StateName);
            UE_LOG(LogRapyutaCore, Error, TEXT("%s"), *SpawnResponse.StatusMessage);
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

    FROSDeleteEntityRequest request;
    deleteEntityService->GetRequest(request);

    UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] Deleting %s"), *GetName(), *request.Name);

    FROSDeleteEntityResponse response;
    response.bSuccess = false;
    if (ServerSimState->Entities.Contains(request.Name))
    {
        // RPC to server
        ServerDeleteEntity(request);
        response.bSuccess = true;
        response.StatusMessage = FString::Printf(TEXT("[%s] Deleted Entity named %s"), *GetName(), *request.Name);
        UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *response.StatusMessage);
    }
    else
    {
        response.StatusMessage = FString::Printf(
            TEXT("[%s] Failed to delete entity named %s. %s do not exist."), *GetName(), *request.Name, *request.Name);
        UE_LOG(LogRapyutaCore, Error, TEXT("%s"), *response.StatusMessage);
    }

    deleteEntityService->SetResponse(response);
}

void URRROS2SimulationStateClient::ServerDeleteEntity_Implementation(const FROSDeleteEntityRequest& InRequest)
{
    ServerSimState->ServerDeleteEntity(InRequest);
}

void URRROS2SimulationStateClient::ServerAddEntity_Implementation(AActor* InEntity)
{
    ServerSimState->ServerAddEntity(InEntity);
}
