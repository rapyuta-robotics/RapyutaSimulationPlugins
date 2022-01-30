// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/SimulationState.h"

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

// Sets default values
ASimulationState::ASimulationState()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

void ASimulationState::Init(AROS2Node* InROS2Node)
{
    ROSServiceNode = InROS2Node;

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
        Entities.Emplace(Entity->GetName(), Entity);
    }
}

void ASimulationState::AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities)
{
    for (auto& Elem :InSpawnableEntities)
    {
        SpawnableEntities.Emplace(Elem.Key, Elem.Value);
    }
}

template<typename T>
bool ASimulationState::CheckEntity(TMap<FString, T> InEntities, const FString & InEntityName, const bool AllowEmpty)
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
            UE_LOG(LogRapyutaCore, Warning, TEXT("Request name %s entity gets invalid -> removed from Entities"), *InEntityName);
            InEntities.Remove(InEntityName);
        }
    }
    else if (AllowEmpty && InEntityName.IsEmpty()){
        result = true;
    }
    else
    {
        UE_LOG(LogRapyutaCore, Warning, 
            TEXT("%s is not under SimulationState control. Please call dedicated method to make Actors under "
                            "SimulationState control."),
            *InEntityName);
    }

    return result;
}

bool ASimulationState::CheckEntity(const FString & InEntityName, const bool AllowEmpty)
{
    return CheckEntity<AActor*>(Entities, InEntityName, AllowEmpty);
}

bool ASimulationState::CheckSpawnableEntity(const FString & InEntityName, const bool AllowEmpty)
{
    return CheckEntity<TSubclassOf<AActor>>(SpawnableEntities, InEntityName, AllowEmpty);
}

void ASimulationState::GetEntityStateSrv(UROS2GenericSrv* Service)
{
    UROS2GetEntityStateSrv* GetEntityStateService = Cast<UROS2GetEntityStateSrv>(Service);

    FROSGetEntityState_Request Request;
    GetEntityStateService->GetRequest(Request);

    // UE_LOG(LogTemp, Warning, TEXT("GetEntityStateSrv called - Currently ignoring Twist"));

    FROSGetEntityState_Response Response;
    Response.success = false;

    if (CheckEntity(Request.name, false)) 
    {
        FVector RefPos = FVector::ZeroVector;
        FQuat RefQuat = FQuat::Identity;
        Response.state_reference_frame.Reset();

        AActor* Entity = Entities[Request.name];
        Response.state_name = Request.name;
 

        if (CheckEntity(Request.reference_frame, true)) 
        {
            Response.success = true;
            FTransform RelativeTrans = Entity->GetTransform();
            if (!Request.reference_frame.IsEmpty())
            {
                AActor* Ref = Entities[Request.reference_frame];
                Response.state_reference_frame = Request.reference_frame;
                RelativeTrans = URRGeneralUtils::RelativeTransform(Ref->GetTransform(), RelativeTrans);

            }
            RelativeTrans = ConversionUtils::TransformUEToROS(RelativeTrans);

            Response.state_pose_position_x = RelativeTrans.GetTranslation().X;
            Response.state_pose_position_y = RelativeTrans.GetTranslation().Y;
            Response.state_pose_position_z = RelativeTrans.GetTranslation().Z;
            Response.state_pose_orientation = RelativeTrans.GetRotation();

            Response.state_twist_linear = FVector::ZeroVector;
            Response.state_twist_angular = FVector::ZeroVector;
        }
    }

    GetEntityStateService->SetResponse(Response);
}

void ASimulationState::SetEntityStateSrv(UROS2GenericSrv* Service)
{
    UROS2SetEntityStateSrv* SetEntityStateService = Cast<UROS2SetEntityStateSrv>(Service);

    FROSSetEntityState_Request Request;
    SetEntityStateService->GetRequest(Request);

    // UE_LOG(LogTemp, Warning, TEXT("SetEntityStateService called - Currently ignoring Twist"));

    FROSSetEntityState_Response Response;
    Response.success = false;

    if (CheckEntity(Request.state_name, false)) 
    {
        if (CheckEntity(Request.state_reference_frame, true)) 
        {
            Response.success = true;
            FVector Pos(Request.state_pose_position_x, Request.state_pose_position_y, Request.state_pose_position_z);
            FTransform WorldTrans(Request.state_pose_orientation, Pos, FVector::OneVector);
            WorldTrans = ConversionUtils::TransformROSToUE(WorldTrans);
            if (!Request.state_reference_frame.IsEmpty())
            {
                AActor* Ref = Entities[Request.state_reference_frame];
                WorldTrans = URRGeneralUtils::WorldTransform(Ref->GetTransform(), WorldTrans);
            }

            AActor* Entity = Entities[Request.state_name];
            Entity->SetActorTransform(WorldTrans);
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
    if (Entities.Contains(Request.name1) && Entities.Contains(Request.name2))
    {
        Response.success = true;
        AActor* Entity1 = Entities[Request.name1];
        AActor* Entity2 = Entities[Request.name2];

        if (false == IsValid(Entity1))
        {
            Entities.Remove(Request.name1);
            UE_LOG(LogRapyutaCore, Warning, TEXT("AttachSrv(): Entity %s gets invalid -> removed from Entities"), *Request.name1);
        }
        else if (false == IsValid(Entity2))
        {
            Entities.Remove(Request.name2);
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
    Response.success = false;

    if (CheckSpawnableEntity(Request.xml, false)) 
    { 
        if (CheckEntity(Request.state_reference_frame, true)) 
        {
            Response.success = true;
            FVector Pos(Request.state_pose_position_x, Request.state_pose_position_y, Request.state_pose_position_z);
            FTransform WorldTrans(Request.state_pose_orientation, Pos, FVector::OneVector);
            if (!Request.state_reference_frame.IsEmpty())
            {
                AActor* Ref = Entities[Request.state_reference_frame];
                WorldTrans = URRGeneralUtils::WorldTransform(Ref->GetTransform(), WorldTrans);
            }
            WorldTrans = ConversionUtils::TransformROSToUE(WorldTrans);

            // todo: check data.name is valid
            // todo: check same name object is exists or not.

            UE_LOG(LogRapyutaCore, Warning, TEXT("Spawning %s"), *Request.xml);

            AActor* NewEntity = GetWorld()->SpawnActorDeferred<AActor>(SpawnableEntities[Request.xml], WorldTrans);
            UROS2Spawnable* SpawnableComponent = NewObject<UROS2Spawnable>(NewEntity, FName("ROS2 Spawn Parameters"));

            SpawnableComponent->RegisterComponent();
            SpawnableComponent->InitializeParameters(Request);
            NewEntity->AddInstanceComponent(SpawnableComponent);
#if WITH_EDITOR
            NewEntity->SetActorLabel(*Request.state_name);
#endif
            NewEntity->Rename(*Request.state_name);

            UGameplayStatics::FinishSpawningActor(NewEntity, WorldTrans);
            AddEntity(NewEntity);

            UE_LOG(LogRapyutaCore, Warning, TEXT("New Spawned Entity Name: %s"), *NewEntity->GetName());

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
    if (Entities.Contains(Request.name))
    {
        AActor* Removed = Entities.FindAndRemoveChecked(Request.name);
        Removed->Destroy();
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Entity %s not found"), *Name);
    }

    DeleteEntityService->SetResponse(Response);
}
