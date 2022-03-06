// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/SimulationState.h"
#include "Tools/SimulationStateData.h"
#include "Tools/ROS2Spawnable.h"
// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"

// rclUE
#include "Srvs/ROS2AttachSrv.h"
#include "Srvs/ROS2DeleteEntitySrv.h"
#include "Srvs/ROS2GetEntityStateSrv.h"
#include "Srvs/ROS2SetEntityStateSrv.h"
#include "Srvs/ROS2SpawnEntitySrv.h"
#include "Tools/RRROS2PlayerController.h"

// Sets default values
ASimulationState::ASimulationState()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

void ASimulationState::Init(AROS2Node* InROS2Node, ASimulationStateData* InSimulationStateData)
{
    ROSServiceNode = InROS2Node;
    SimulationStateData = InSimulationStateData;
    // register delegates to node
    FServiceCallback GetEntityStateSrvCallback;
    FServiceCallback SetEntityStateSrvCallback;
    FServiceCallback AttachSrvCallback;
    FServiceCallback SpawnEntitySrvCallback;
    FServiceCallback DeleteEntitySrvCallback;
    GetEntityStateSrvCallback.BindUObject(this, &ASimulationState::GetEntityStateSrv);
    SetEntityStateSrvCallback.BindUObject(this, &ASimulationState::SetEntityStateSrv);
    AttachSrvCallback.BindUObject(this, &ASimulationState::AttachSrv);
    SpawnEntitySrvCallback.BindUObject(this, &ASimulationState::SpawnEntitySrv);
    DeleteEntitySrvCallback.BindUObject(this, &ASimulationState::DeleteEntitySrv);
    ROSServiceNode->AddService(TEXT("GetEntityState"), UROS2GetEntityStateSrv::StaticClass(), GetEntityStateSrvCallback);
    ROSServiceNode->AddService(TEXT("SetEntityState"), UROS2SetEntityStateSrv::StaticClass(), SetEntityStateSrvCallback);
    ROSServiceNode->AddService(TEXT("Attach"), UROS2AttachSrv::StaticClass(), AttachSrvCallback);
    ROSServiceNode->AddService(TEXT("SpawnEntity"), UROS2SpawnEntitySrv::StaticClass(), SpawnEntitySrvCallback);
    ROSServiceNode->AddService(TEXT("DeleteEntity"), UROS2DeleteEntitySrv::StaticClass(), DeleteEntitySrvCallback);

    // add all actors
#if WITH_EDITOR
    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), AllActors);
    UE_LOG(LogTemp, Warning, TEXT("Found %d actors in the scene"), AllActors.Num());
#endif
    for (TActorIterator<AActor> It(GetWorld(), AActor::StaticClass()); It; ++It)
    {
        AActor* actor = *It;
        AddEntity(actor);
    }
}

void ASimulationState::AddEntity(AActor* Entity)
{
    if (IsValid(Entity))
    {
        SimulationStateData->AddEntity(Entity);
    }
}

void ASimulationState::AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities)
{
    for (auto& Elem :InSpawnableEntities)
    {
        SpawnableEntities.Emplace(Elem.Key, Elem.Value);
    }
}

void ASimulationState::GetEntityStateSrv(UROS2GenericSrv* Service)
{
    UROS2GetEntityStateSrv* GetEntityStateService = Cast<UROS2GetEntityStateSrv>(Service);

    FROSGetEntityState_Request Request;
    GetEntityStateService->GetRequest(Request);

    // UE_LOG(LogTemp, Warning, TEXT("GetEntityStateSrv called - Currently ignoring Twist"));

    FROSGetEntityState_Response Response;
    Response.success = false;
    if (SimulationStateData->Entities.Contains(Request.name))
    {
        if (IsValid(SimulationStateData->Entities[Request.name]))
        {
            Response.success = true;
            FVector RefPos = FVector::ZeroVector;
            FQuat RefQuat = FQuat::Identity;
            Response.state_reference_frame.Reset();
            if (SimulationStateData->Entities.Contains(Request.reference_frame))
            {
                if (IsValid(SimulationStateData->Entities[Request.reference_frame]))
                {
                    AActor* Ref = SimulationStateData->Entities[Request.reference_frame];
                    RefPos = Ref->GetActorLocation() / 100.f;
                    RefQuat = Ref->GetActorQuat();
                    Response.state_reference_frame = Request.reference_frame;
                }
                else
                {
                    SimulationStateData->Entities.Remove(Request.reference_frame);
                    UE_LOG(LogRapyutaCore,
                           Warning,
                           TEXT("Reference frame %s entity gets invalid -> removed from Entities"),
                           *Request.reference_frame);
                }
            }
            else
            {
                UE_LOG(LogRapyutaCore, Warning, TEXT("Reference frame %s entity not found"), *Request.reference_frame);
            }
            AActor* Entity = SimulationStateData->Entities[Request.name];
            Response.state_name = Request.name;
            FVector Pos = RefQuat.Inverse().RotateVector(Entity->GetActorLocation() / 100.f - RefPos);
            Response.state_pose_position_x = Pos.X;
            Response.state_pose_position_y = Pos.Y;
            Response.state_pose_position_z = Pos.Z;
            Response.state_pose_orientation = Entity->GetActorQuat() * RefQuat.Inverse();
            Response.state_pose_orientation.Normalize();
            LeftToRight(Response.state_pose_position_x,
                        Response.state_pose_position_y,
                        Response.state_pose_position_z,
                        Response.state_pose_orientation);
            Response.state_twist_linear = FVector::ZeroVector;
            Response.state_twist_angular = FVector::ZeroVector;
        }
        else
        {
            SimulationStateData->Entities.Remove(Request.name);
            UE_LOG(LogRapyutaCore, Warning, TEXT("Request name %s entity gets invalid -> removed from Entities"), *Request.name);
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("Request name %s entity not found"), *Request.name);
    }

    GetEntityStateService->SetResponse(Response);
}

// todo should be in the common utility fuctions
void ASimulationState::LeftToRight(double& pos_x, double& pos_y, double& pos_z, FQuat& orientation)
{
    pos_y = -pos_y;
    orientation.X = -orientation.X;
    orientation.Z = -orientation.Z;
}
void ASimulationState::LeftToRight(FVector& position, FQuat& orientation)
{
    position.Y = -position.Y;
    orientation.X = -orientation.X;
    orientation.Z = -orientation.Z;
}

bool ASimulationState::ReferenceFrameToInertiaFrame(const FString& InReferenceFrame,
                                                    double& OutPositionX,
                                                    double& OutPositionY,
                                                    double& OutPositionZ,
                                                    FQuat& OutOrientation)
{
    bool bSuccess = false;
    LeftToRight(OutPositionX, OutPositionY, OutPositionZ, OutOrientation);
    if (InReferenceFrame.IsEmpty())
    {
        bSuccess = true;
        OutPositionX *= 100.;
        OutPositionY *= 100.;
        OutPositionZ *= 100.;
    }
    else if (SimulationStateData->Entities.Contains(InReferenceFrame))
    {
        if (IsValid(SimulationStateData->Entities[InReferenceFrame]))
        {
            bSuccess = true;
            AActor* ref = SimulationStateData->Entities[InReferenceFrame];
            FVector refPos = ref->GetActorLocation();
            FQuat refQuat = ref->GetActorQuat();
            FVector OutputVec = FVector(OutPositionX, OutPositionY, OutPositionZ);

            OutputVec = refQuat.RotateVector(OutputVec);
            OutPositionX = refPos.X + OutputVec.X * 100.;
            OutPositionY = refPos.Y + OutputVec.Y * 100.;
            OutPositionZ = refPos.Z + OutputVec.Z * 100.;
            OutOrientation *= refQuat;
        }
        else
        {
            bSuccess = false;
            SimulationStateData->Entities.Remove(InReferenceFrame);
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("ReferenceFrameToInertiaFrame(): InReferenceFrame %s entity gets invalid -> Removed from Entities"),
                   *InReferenceFrame);
        }
    }
    else
    {
        bSuccess = false;
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("ReferenceFrameToInertiaFrame(): InReferenceFrame %s entity not found"),
               *InReferenceFrame);
    }

    return bSuccess;
}

void ASimulationState::SetEntityStateSrv(UROS2GenericSrv* Service)
{
    UROS2SetEntityStateSrv* SetEntityStateService = Cast<UROS2SetEntityStateSrv>(Service);

    FROSSetEntityState_Request Request;
    SetEntityStateService->GetRequest(Request);

    // UE_LOG(LogTemp, Warning, TEXT("SetEntityStateService called - Currently ignoring Twist"));

    FROSSetEntityState_Response Response;
    Response.success = ReferenceFrameToInertiaFrame(Request.state_reference_frame,
                                                    Request.state_pose_position_x,
                                                    Request.state_pose_position_y,
                                                    Request.state_pose_position_z,
                                                    Request.state_pose_orientation);
    if (Response.success)
    {
        if (SimulationStateData->Entities.Contains(Request.state_name))
        {
            if (IsValid(SimulationStateData->Entities[Request.state_name]))
            {
                AActor* Entity = SimulationStateData->Entities[Request.state_name];
                FVector Pos(Request.state_pose_position_x, Request.state_pose_position_y, Request.state_pose_position_z);
                Entity->SetActorLocationAndRotation(Pos, Request.state_pose_orientation);
            }
            else
            {
                Response.success = false;
                SimulationStateData->Entities.Remove(Request.state_name);
                UE_LOG(LogRapyutaCore,
                       Warning,
                       TEXT("SetEntityStateSrv(): Entity %s gets invalid -> removed from Entities"),
                       *Request.state_name);
            }
        }
        else
        {
            Response.success = false;
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("Entity %s not exit or not under SimulationState control. Please call AddEntity to make Actors under "
                        "SimulationState control."),
                   *Request.state_name);
        }
    }
    SetEntityStateService->SetResponse(Response);
}

void ASimulationState::AttachSrv(UROS2GenericSrv* Service)
{
    UROS2AttachSrv* AttachService = Cast<UROS2AttachSrv>(Service);

    FROSAttach_Request Request;
    AttachService->GetRequest(Request);

    FROSAttach_Response Response;
    Response.success = false;
    if (SimulationStateData->Entities.Contains(Request.name1) && SimulationStateData->Entities.Contains(Request.name2))
    {
        Response.success = true;
        AActor* Entity1 = SimulationStateData->Entities[Request.name1];
        AActor* Entity2 = SimulationStateData->Entities[Request.name2];

        if (false == IsValid(Entity1))
        {
            SimulationStateData->Entities.Remove(Request.name1);
            UE_LOG(LogRapyutaCore, Warning, TEXT("AttachSrv(): Entity %s gets invalid -> removed from Entities"), *Request.name1);
        }
        else if (false == IsValid(Entity2))
        {
            SimulationStateData->Entities.Remove(Request.name2);
            UE_LOG(LogRapyutaCore, Warning, TEXT("AttachSrv(): Entity %s gets invalid -> removed from Entities"), *Request.name2);
        }
        else
        {
            if (!Entity2->IsAttachedTo(Entity1))
            {
                Entity2->AttachToActor(Entity1, FAttachmentTransformRules::KeepWorldTransform);
            }
            else
            {
                Entity2->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            }
        }
    }
    else
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("Entity %s and/or %s not exit or not under SimulationState Actor control. Please call AddEntity to make Actors "
                    "under SimulationState control."),
               *Request.name1,
               *Request.name2);
    }

    AttachService->SetResponse(Response);
}

void ASimulationState::SpawnEntitySrv(UROS2GenericSrv* Service)
{
    UROS2SpawnEntitySrv* SpawnEntityService = Cast<UROS2SpawnEntitySrv>(Service);

    FROSSpawnEntity_Request Request;
    SpawnEntityService->GetRequest(Request);

    FROSSpawnEntity_Response Response;

    Response.success = ReferenceFrameToInertiaFrame(Request.state_reference_frame,
                                                    Request.state_pose_position_x,
                                                    Request.state_pose_position_y,
                                                    Request.state_pose_position_z,
                                                    Request.state_pose_orientation);

    if (Response.success)
    {
        if (SpawnableEntities.Contains(Request.xml))
        {
            if (IsValid(SpawnableEntities[Request.xml]))
            {
                // todo: check data.name is valid
                // todo: check same name object is exists or not.

                UE_LOG(LogRapyutaCore, Warning, TEXT("Spawning %s"), *Request.xml);
                Response.success = true;

                FRotator Rotator = Request.state_pose_orientation.Rotator();
                FVector Position(Request.state_pose_position_x, Request.state_pose_position_y, Request.state_pose_position_z);
                FVector Scale(1, 1, 1);
                FTransform Transform(Rotator, Position, Scale);

                AActor* NewEntity = GetWorld()->SpawnActorDeferred<AActor>(SpawnableEntities[Request.xml], Transform);
                UROS2Spawnable* SpawnableComponent = NewObject<UROS2Spawnable>(NewEntity, FName("ROS2 Spawn Parameters"));

                SpawnableComponent->RegisterComponent();
                SpawnableComponent->InitializeParameters(Request);
                NewEntity->AddInstanceComponent(SpawnableComponent);
#if WITH_EDITOR
                NewEntity->SetActorLabel(*Request.state_name);
#endif
                NewEntity->Rename(*Request.state_name);

                UGameplayStatics::FinishSpawningActor(NewEntity, Transform);
                AddEntity(NewEntity);

                ARRROS2GameMode *ros2GameMode = Cast<ARRROS2GameMode>(UGameplayStatics::GetGameMode(GetWorld()));


                for (APlayerController* Player : ros2GameMode->ClientControllerList)
                {
                    if(Cast<ARRROS2PlayerController>(Player)->PlayerName == *NewEntity->GetName()) {
                        Player->Possess(Cast<APawn>(NewEntity));
                    }
                }

                UE_LOG(LogRapyutaCore, Warning, TEXT("New Spawned Entity Name: %s"), *NewEntity->GetName());
            }
            else
            {
                SimulationStateData->Entities.Remove(Request.xml);
                UE_LOG(LogRapyutaCore,
                       Warning,
                       TEXT("SpawnEntitySrv(): Entity %s gets invalid -> removed from Entities"),
                       *Request.xml);
            }
        }
        else
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("Entity %s not exit or not under SimulationState Actor control. Please call AddEntity to make Actors under "
                        "SimulationState control."),
                   *Request.xml);
        }
    }

    SpawnEntityService->SetResponse(Response);
}

void ASimulationState::DeleteEntitySrv(UROS2GenericSrv* Service)
{
    UROS2DeleteEntitySrv* DeleteEntityService = Cast<UROS2DeleteEntitySrv>(Service);

    FString Name;
    FROSDeleteEntity_Request Request;
    DeleteEntityService->GetRequest(Request);

    UE_LOG(LogTemp, Warning, TEXT("DeleteEntityService called"));

    FROSDeleteEntity_Response Response;
    Response.success = false;
    if (SimulationStateData->Entities.Contains(Request.name))
    {
        AActor* Removed = SimulationStateData->Entities.FindAndRemoveChecked(Request.name);
        Removed->Destroy();
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Entity %s not found"), *Name);
    }

    DeleteEntityService->SetResponse(Response);
}
