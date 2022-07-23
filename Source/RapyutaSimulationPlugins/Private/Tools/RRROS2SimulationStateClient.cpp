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
    ClientROS2Node = InROS2Node;

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
    DOREPLIFETIME(URRROS2SimulationStateClient, ClientROS2Node);
    DOREPLIFETIME(URRROS2SimulationStateClient, ServerSimState);
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
            UE_LOG(LogRapyutaCore, Warning, TEXT("Entity named [%s] gets invalid -> removed from Entities"), *InEntityName);
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
               TEXT("Entity named [%s] is not under SimulationState control. Please register it to SimulationState!"),
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
    return CheckEntity<TSubclassOf<AActor>>(ServerSimState->SpawnableEntities, InEntityName, bAllowEmpty);
}

void URRROS2SimulationStateClient::GetEntityStateSrv(UROS2GenericSrv* InService)
{
    UROS2GetEntityStateSrv* GetEntityStateService = Cast<UROS2GetEntityStateSrv>(InService);

    FROSGetEntityState_Request request;
    GetEntityStateService->GetRequest(request);

    FROSGetEntityState_Response response;
    response.state_name = request.name;
    response.success = CheckEntity(request.name, false) && CheckEntity(request.reference_frame, true);

    if (response.success)
    {
        FTransform relativeTransf;
        FTransform worldTransf = ServerSimState->Entities[request.name]->GetTransform();
        URRGeneralUtils::GetRelativeTransform(
            request.reference_frame, ServerSimState->Entities.FindRef(request.reference_frame), worldTransf, relativeTransf);
        relativeTransf = URRConversionUtils::TransformUEToROS(relativeTransf);

        response.state_pose_position_x = relativeTransf.GetTranslation().X;
        response.state_pose_position_y = relativeTransf.GetTranslation().Y;
        response.state_pose_position_z = relativeTransf.GetTranslation().Z;
        response.state_pose_orientation = relativeTransf.GetRotation();

        response.state_twist_linear = FVector::ZeroVector;
        response.state_twist_angular = FVector::ZeroVector;
    }

    GetEntityStateService->SetResponse(response);
}

void URRROS2SimulationStateClient::SetEntityStateSrv(UROS2GenericSrv* InService)
{
    UROS2SetEntityStateSrv* setEntityStateService = Cast<UROS2SetEntityStateSrv>(InService);

    FROSSetEntityState_Request request;
    setEntityStateService->GetRequest(request);

    FROSSetEntityState_Response response;
    response.success = CheckEntity(request.state_name, false) && CheckEntity(request.state_reference_frame, true);

    if (response.success)
    {
        ServerSimState->ServerSetEntityState(request);
    }

    setEntityStateService->SetResponse(response);
}

void URRROS2SimulationStateClient::AttachSrv(UROS2GenericSrv* InService)
{
    UROS2AttachSrv* attachService = Cast<UROS2AttachSrv>(InService);

    FROSAttach_Request request;
    attachService->GetRequest(request);

    FROSAttach_Response response;
    response.success = CheckEntity(request.name1, false) && CheckEntity(request.name2, false);
    if (response.success)
    {
        ServerSimState->ServerAttach(request);
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("Entity %s and/or %s not exit or not under SimulationState Actor control. Please call AddEntity to make Actors "
                    "under SimulationState control."),
               *request.name1,
               *request.name2);
    }

    attachService->SetResponse(response);
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
            // Spawn entity
            AActor* newEntity = ServerSimState->ServerSpawnEntity(request);
            response.bSuccess = (nullptr != newEntity);
            response.StatusMessage = newEntity
                                       ? FString::Printf(TEXT("Newly spawned Entity: %s"), *newEntity->GetName())
                                       : FString::Printf(TEXT("[%s]Entity spawning failed, probably out collision!"), *entityName);
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *response.StatusMessage);
        }
        else
        {
            response.bSuccess = false;
            response.StatusMessage =
                FString::Printf(TEXT("Entity spawning failed - [%s] given name actor already exists!"), *entityName);
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
    for (uint32 i = 0; i < entityListRequest.NameList.Num(); ++i)
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("Spawning Entity : %s (name: %s)"),
               *entityListRequest.TypeList[i],
               *entityListRequest.NameList[i]);
        FROSSpawnEntityRequest entityRequest;
        entityRequest.Xml = entityListRequest.TypeList[i];
        entityRequest.RobotNamespace = EMPTY_STR;
        entityRequest.StateName = entityListRequest.NameList[i];
        entityRequest.StatePosePositionX = entityListRequest.PositionList[i].X;
        entityRequest.StatePosePositionY = entityListRequest.PositionList[i].Y;
        entityRequest.StatePosePositionZ = entityListRequest.PositionList[i].Z;
        entityRequest.StatePoseOrientation = entityListRequest.OrientationList[i];
        entityRequest.StateTwistLinear = entityListRequest.TwistLinearList[i];
        entityRequest.StateTwistAngular = entityListRequest.TwistAngularList[i];
        entityRequest.StateReferenceFrame = entityListRequest.ReferenceFrameList[i];
        entityRequest.Tags.Add(entityListRequest.TagsList[i]);

        AActor* newEntity = nullptr;
        if (CheckSpawnableEntity(entityRequest.Xml, false) && CheckEntity(entityRequest.StateReferenceFrame, true))
        {
            if (nullptr == URRUObjectUtils::FindActorByName<AActor>(GetWorld(), entityRequest.StateName))
            {
                newEntity = ServerSimState->ServerSpawnEntity(entityRequest);
                if (newEntity)
                {
                    numEntitySpawned++;
                }
            }
            else
            {
                UE_LOG(LogRapyutaCore,
                       Error,
                       TEXT("Entity spawning failed - Actor of [%s] name already exists"),
                       *entityRequest.StateName);
            }
        }

        statusMessage.Append(newEntity ? FString::Printf(TEXT("%s,"), *newEntity->GetName())
                                       : FString::Printf(TEXT("%s,"), *entityRequest.StateName));
    }

    FROSSpawnEntitiesResponse entityListResponse;
    entityListResponse.bSuccess = (numEntitySpawned == entityListRequest.NameList.Num());
    entityListResponse.StatusMessage = entityListResponse.bSuccess
                                         ? FString::Printf(TEXT("Newly spawned entities: [%s]"), *statusMessage)
                                         : FString::Printf(TEXT("Failed to be spawned entities: [%s]"), *statusMessage);

    spawnEntitiesService->SetResponse(entityListResponse);
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

    FROSDeleteEntity_Request request;
    deleteEntityService->GetRequest(request);

    UE_LOG(LogTemp, Warning, TEXT("DeleteEntityService called"));

    FROSDeleteEntity_Response response;
    response.success = false;
    if (ServerSimState->Entities.Contains(request.name))
    {
        ServerSimState->ServerDeleteEntity(request);
        response.success = true;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Entity %s not found"), *request.name);
    }

    deleteEntityService->SetResponse(response);
}
