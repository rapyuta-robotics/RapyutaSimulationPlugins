// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/SimulationState.h"

#include "Tools/ROS2Spawnable.h"
// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include <TimerManager.h>

// rclUE
#include "Srvs/ROS2AttachSrv.h"
#include "Srvs/ROS2DeleteEntitySrv.h"
#include "Srvs/ROS2GetEntityStateSrv.h"
#include "Srvs/ROS2SetEntityStateSrv.h"
#include "Srvs/ROS2SpawnEntitySrv.h"
#include "Tools/SimulationStateClient.h"

// RapyutaSimulationPlugins
#include "Core/RRUObjectUtils.h"

// Sets default values
USimulationStateClient::USimulationStateClient()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
//    bReplicates = true;
//    PrimaryActorTick.bCanEverTick = true;
//    bAlwaysRelevant=true;
}

void USimulationStateClient::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME( USimulationStateClient, SimulationState );
    DOREPLIFETIME( USimulationStateClient, SpawnResponse );
}

void USimulationStateClient::InitSimulationState()
{
    SimulationState = CastChecked<ASimulationState>(UGameplayStatics::GetActorOfClass(GetWorld(), ASimulationState::StaticClass()));
}

void USimulationStateClient::InitROS2Node(AROS2Node* InROS2Node)
{
    ROSServiceNode = InROS2Node;

    // register delegates to node
    FServiceCallback GetEntityStateSrvCallback;
    FServiceCallback SetEntityStateSrvCallback;
    FServiceCallback AttachSrvCallback;
    FServiceCallback SpawnEntitySrvCallback;
    FServiceCallback DeleteEntitySrvCallback;
    GetEntityStateSrvCallback.BindDynamic(this, &USimulationStateClient::GetEntityStateSrv);
    SetEntityStateSrvCallback.BindDynamic(this, &USimulationStateClient::SetEntityStateSrv);
    AttachSrvCallback.BindDynamic(this, &USimulationStateClient::AttachSrv);
    SpawnEntitySrvCallback.BindDynamic(this, &USimulationStateClient::SpawnEntitySrv);
    DeleteEntitySrvCallback.BindDynamic(this, &USimulationStateClient::DeleteEntitySrv);
    ROSServiceNode->AddServiceServer(TEXT("GetEntityState"), UROS2GetEntityStateSrv::StaticClass(), GetEntityStateSrvCallback);
    ROSServiceNode->AddServiceServer(TEXT("SetEntityState"), UROS2SetEntityStateSrv::StaticClass(), SetEntityStateSrvCallback);
    ROSServiceNode->AddServiceServer(TEXT("Attach"), UROS2AttachSrv::StaticClass(), AttachSrvCallback);
    ROSServiceNode->AddServiceServer(TEXT("SpawnEntity"), UROS2SpawnEntitySrv::StaticClass(), SpawnEntitySrvCallback);
    ROSServiceNode->AddServiceServer(TEXT("DeleteEntity"), UROS2DeleteEntitySrv::StaticClass(), DeleteEntitySrvCallback);
}


void USimulationStateClient::GetEntityStateSrv(UROS2GenericSrv* Service)
{
    UROS2GetEntityStateSrv* GetEntityStateService = Cast<UROS2GetEntityStateSrv>(Service);

    FROSGetEntityState_Request Request;
    GetEntityStateService->GetRequest(Request);

    // UE_LOG(LogTemp, Warning, TEXT("GetEntityStateSrv called - Currently ignoring Twist"));

    FROSGetEntityState_Response Response;
    Response.state_name = Request.name;
    Response.success = SimulationState->CheckEntity(Request.name, false) && SimulationState->CheckEntity(Request.reference_frame, true);

    if (Response.success)
    {
        FTransform relativeTransf;
        FTransform worldTransf = SimulationState->Entities[Request.name]->GetTransform();
        URRGeneralUtils::GetRelativeTransform(
                Request.reference_frame,
                SimulationState->Entities.Contains(Request.reference_frame) ? SimulationState->Entities[Request.reference_frame] : nullptr,
                worldTransf,
                relativeTransf);
        relativeTransf = ConversionUtils::TransformUEToROS(relativeTransf);

        Response.state_pose_position_x = relativeTransf.GetTranslation().X;
        Response.state_pose_position_y = relativeTransf.GetTranslation().Y;
        Response.state_pose_position_z = relativeTransf.GetTranslation().Z;
        Response.state_pose_orientation = relativeTransf.GetRotation();

        Response.state_twist_linear = FVector::ZeroVector;
        Response.state_twist_angular = FVector::ZeroVector;
    }

    GetEntityStateService->SetResponse(Response);
}


void USimulationStateClient::SetEntityStateSrv(UROS2GenericSrv* Service)
{
    UROS2SetEntityStateSrv* SetEntityStateService = Cast<UROS2SetEntityStateSrv>(Service);

    FROSSetEntityState_Request Request;
    SetEntityStateService->GetRequest(Request);

    // UE_LOG(LogTemp, Warning, TEXT("SetEntityStateService called - Currently ignoring Twist"));

    FROSSetEntityState_Response Response;
    Response.success = SimulationState->CheckEntity(Request.state_name, false) && SimulationState->CheckEntity(Request.state_reference_frame, true);

    if (Response.success)
    {
        ServerSetEntityState(Request);
    }

    SetEntityStateService->SetResponse(Response);
}

void USimulationStateClient::ServerSetEntityState_Implementation(FROSSetEntityState_Request Request) {
    SimulationState->ServerSetEntityState(Request);
}

void USimulationStateClient::AttachSrv(UROS2GenericSrv* Service)
{
    UROS2AttachSrv* AttachService = Cast<UROS2AttachSrv>(Service);

    FROSAttach_Request Request;
    AttachService->GetRequest(Request);

    FROSAttach_Response Response;
    Response.success = SimulationState->CheckEntity(Request.name1, false) && SimulationState->CheckEntity(Request.name2, false);
    if (Response.success)
    {
        ServerAttach(Request);
    }
    else
    {
        UE_LOG(
                LogRapyutaCore,
                Warning,
                TEXT(
                        "Entity %s and/or %s not existing or not under SimulationState Actor control. Please call AddEntity to make Actors "
                        "under SimulationState control."),
                *Request.name1,
                *Request.name2);
    }

    AttachService->SetResponse(Response);
}

void USimulationStateClient::ServerAttach_Implementation(FROSAttach_Request Request)
{
   SimulationState->ServerAttach(Request);
}

void USimulationStateClient::SpawnEntitySrv(UROS2GenericSrv* Service)
{
    FROSSpawnEntityResponse Response;
    SpawnResponse = Response;

    SpawnEntityService = nullptr;
    SpawnEntityService = Cast<UROS2SpawnEntitySrv>(Service);

    FROSSpawnEntityRequest Request;
    SpawnEntityService->GetRequest(Request);

    Response.bSuccess = SimulationState->CheckSpawnableEntity(Request.Xml, false) && SimulationState->CheckEntity(Request.StateReferenceFrame, true);
    if (Response.bSuccess)
    {
        const FString& entityModelName = Request.Xml;
        const FString& entityName = Request.StateName;
        verify(false == entityName.IsEmpty());
        if (nullptr == URRUObjectUtils::FindActorByName<AActor>(GetWorld(), entityName))
        {
            ServerSpawnEntity(Request);
            Response.StatusMessage = FString::Printf(TEXT("Newly spawned Entity: %s"), *entityName);
            UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *Response.StatusMessage);
        }
        else
        {
            Response.bSuccess = false;
            Response.StatusMessage =
                    FString::Printf(TEXT("Entity spawning failed - [%s] given name actor already exists!"), *entityName);
            UE_LOG(LogRapyutaCore, Error, TEXT("%s"), *Response.StatusMessage);
        }
    }
    SpawnEntityService->SetResponse(Response);


}

void USimulationStateClient::ServerSpawnEntity_Implementation(FROSSpawnEntityRequest Request)
{
    SimulationState->ServerSpawnEntity(Request);
}

////Currently this code doesnt seem to trigger the ROS2 Service Response... keeping this in since if
//// that can be figured out, we can have better verification of spawned actors
////Code to do checking of if Entity is spawned using 2 timers, one for timeout and one for triggering this every x s
//void USimulationStateClient::SpawnEntityCheck()
//{
//    if (GetNetMode() == NM_Client) {
//        FROSSpawnEntityRequest Request;
//        SpawnEntityService->GetRequest(Request);
//
//        AActor *MatchingEntity;
//        for (AActor *Entity: SimulationState->EntityList) {
//            if (Entity) {
//                UROS2Spawnable *rosSpawnParameters = Entity->FindComponentByClass<UROS2Spawnable>();
//                if (rosSpawnParameters) {
//                    if (rosSpawnParameters->GetName() == Request.StateName) {
//                        MatchingEntity = Entity;
//                    }
//                }
//            }
//        }
//        if (MatchingEntity) {
//            SpawnResponse.bSuccess = true;
//            SpawnResponse.StatusMessage = FString::Printf(TEXT("Newly spawned Entity: %s"), *Request.StateName);
//            UE_LOG(LogRapyutaCore, Warning, TEXT("%s"), *SpawnResponse.StatusMessage);
//            SpawnEntityService->SetResponse(SpawnResponse);
//            GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
//        } else {
//            SpawnResponse.bSuccess = false;
//            SpawnResponse.StatusMessage =
//                    FString::Printf(TEXT("Entity spawning failed for actor [%s]"), *Request.StateName);
//            UE_LOG(LogRapyutaCore, Error, TEXT("%s"), *SpawnResponse.StatusMessage);
//        }
//    }
//}
//
//void USimulationStateClient::SpawnEntityResponse()
//{
//    SpawnEntityService->SetResponse(SpawnResponse);
//    GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
//}

void USimulationStateClient::DeleteEntitySrv(UROS2GenericSrv* Service)
{
    UROS2DeleteEntitySrv* DeleteEntityService = Cast<UROS2DeleteEntitySrv>(Service);

    FString Name;
    FROSDeleteEntity_Request Request;
    DeleteEntityService->GetRequest(Request);

    UE_LOG(LogTemp, Warning, TEXT("DeleteEntityService called"));

    FROSDeleteEntity_Response Response;
    Response.success = false;
    if (SimulationState->Entities.Contains(Request.name))
    {
        ServerDeleteEntity(Request);
        Response.success = true;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Entity %s not found"), *Name);
    }

    DeleteEntityService->SetResponse(Response);
}

void USimulationStateClient::ServerDeleteEntity_Implementation(FROSDeleteEntity_Request Request)
{
    SimulationState->ServerDeleteEntity(Request);
}
