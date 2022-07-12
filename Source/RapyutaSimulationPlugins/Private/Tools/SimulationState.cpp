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

// RapyutaSimulationPlugins
#include "Core/RRUObjectUtils.h"

// Sets default values
ASimulationState::ASimulationState()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    bReplicates = true;
    PrimaryActorTick.bCanEverTick = true;
    bAlwaysRelevant=true;
}

void ASimulationState::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME( ASimulationState, EntityList );
    DOREPLIFETIME( ASimulationState, SpawnableEntityList );
    DOREPLIFETIME( ASimulationState, SpawnableEntityNameList );
}

void ASimulationState::Init(AROS2Node* InROS2Node)
{
    InitROS2Node(InROS2Node);
    InitEntities();

}

void ASimulationState::InitEntities()
{
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
    GetWorld()->GetTimerManager().SetTimer(TimerHandle, this,
                                           &ASimulationState::GetSplitSpawnableEntities, 1.0f,
                                           true);
}


void ASimulationState::InitROS2Node(AROS2Node* InROS2Node)
{
    ROSServiceNode = InROS2Node;

    // register delegates to node
//    FServiceCallback GetEntityStateSrvCallback;
//    FServiceCallback SetEntityStateSrvCallback;
//    FServiceCallback AttachSrvCallback;
//    FServiceCallback SpawnEntitySrvCallback;
//    FServiceCallback DeleteEntitySrvCallback;
//    GetEntityStateSrvCallback.BindDynamic(this, &ASimulationState::GetEntityStateSrv);
//    SetEntityStateSrvCallback.BindDynamic(this, &ASimulationState::SetEntityStateSrv);
//    AttachSrvCallback.BindDynamic(this, &ASimulationState::AttachSrv);
//    SpawnEntitySrvCallback.BindDynamic(this, &ASimulationState::SpawnEntitySrv);
//    DeleteEntitySrvCallback.BindDynamic(this, &ASimulationState::DeleteEntitySrv);
//    ROSServiceNode->AddServiceServer(TEXT("GetEntityState"), UROS2GetEntityStateSrv::StaticClass(), GetEntityStateSrvCallback);
//    ROSServiceNode->AddServiceServer(TEXT("SetEntityState"), UROS2SetEntityStateSrv::StaticClass(), SetEntityStateSrvCallback);
//    ROSServiceNode->AddServiceServer(TEXT("Attach"), UROS2AttachSrv::StaticClass(), AttachSrvCallback);
//    ROSServiceNode->AddServiceServer(TEXT("SpawnEntity"), UROS2SpawnEntitySrv::StaticClass(), SpawnEntitySrvCallback);
//    ROSServiceNode->AddServiceServer(TEXT("DeleteEntity"), UROS2DeleteEntitySrv::StaticClass(), DeleteEntitySrvCallback);
}



void ASimulationState::AddEntity(AActor* Entity)
{
    if (IsValid(Entity))
    {
        GetSplitSpawnableEntities();
        Entities.Emplace(Entity->GetName(), Entity);
        EntityList.Emplace(Entity);
        for (auto& tag : Entity->Tags)
        {
            if (EntitiesWithTag.Contains(tag))
            {
                EntitiesWithTag[tag].Actors.Emplace(Entity);
            }
            else
            {
                FActors actors;
                actors.Actors.Emplace(Entity);
                EntitiesWithTag.Emplace(tag, actors);
            }
        }
    }
}

//void ASimulationState::OnRep_Entity() {
//
//}
//Work around to replicating Entities and EntitiesWithTag since TMaps cannot be replicated
void ASimulationState::OnRep_Entity()
{
    for(AActor *Entity: EntityList) {
        if(Entity) {
            if (!Entities.Contains(Entity->GetName())) {
                UROS2Spawnable *rosSpawnParameters = Entity->FindComponentByClass<UROS2Spawnable>();
                if (rosSpawnParameters) {
                    Entities.Emplace(rosSpawnParameters->GetName(), Entity);
                }
                else {
                    Entities.Emplace(Entity->GetName(), Entity);
                }
            }

            for (auto &tag: Entity->Tags) {
                AddTaggedEntities(Entity, tag);
            }

            UROS2Spawnable* EntitySpawnParam = Entity->FindComponentByClass<UROS2Spawnable>();
            if (EntitySpawnParam) {
                Entity->Rename(*EntitySpawnParam->GetName());
                for (auto &tag: EntitySpawnParam->ActorTags) {
                    AddTaggedEntities(Entity, FName(tag));
                }
            }
        }
    }


}


void ASimulationState::OnRep_SpawnableEntity()
{
    for (int i =0; i < SpawnableEntityList.Num(); i++) {
        SpawnableEntities.Emplace(SpawnableEntityNameList[i], SpawnableEntityList[i]);
    }

}
void ASimulationState::AddTaggedEntities(AActor* Entity, const FName& InTag){

    if (EntitiesWithTag.Contains(InTag)) {
        // Check if Actor in EntitiesWithTag

        bool ToEmplace = true;
        for (AActor *TaggedActor: EntitiesWithTag[InTag].Actors) {
            if (TaggedActor->GetName() == Entity->GetName()) {
                ToEmplace = false;
            }
        }
        if (ToEmplace) {
            EntitiesWithTag[InTag].Actors.Emplace(Entity);
        }
    } else {
        FActors actors;
        actors.Actors.Emplace(Entity);
        EntitiesWithTag.Emplace(InTag, actors);
    }
}

void ASimulationState::AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities)
{
    for (auto& Elem : InSpawnableEntities)
    {
        SpawnableEntityList.Emplace(Elem.Value);
        SpawnableEntityNameList.Emplace(Elem.Key);
        SpawnableEntities.Emplace(Elem.Key, Elem.Value);
    }
}
void ASimulationState::GetSplitSpawnableEntities()
{
    for (auto& Elem : SpawnableEntities)
    {
        SpawnableEntityList.Emplace(Elem.Value);
        SpawnableEntityNameList.Emplace(Elem.Key);
    }
    if(SpawnableEntityList.Num() > 0) {
        GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
    }
}
template<typename T>
bool ASimulationState::CheckEntity(TMap<FString, T>& InEntities, const FString& InEntityName, const bool bAllowEmpty)
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
template bool ASimulationState::CheckEntity<AActor*>(TMap<FString, AActor*>& InEntities,
                                                     const FString& InEntityName,
                                                     const bool bAllowEmpty);

template bool ASimulationState::CheckEntity<TSubclassOf<AActor>>(TMap<FString, TSubclassOf<AActor>>& InEntities,
                                                                 const FString& InEntityName,
                                                                 const bool bAllowEmpty);

bool ASimulationState::CheckEntity(const FString& InEntityName, const bool bAllowEmpty)
{
    return CheckEntity<AActor*>(Entities, InEntityName, bAllowEmpty);
}

bool ASimulationState::CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty)
{
    return CheckEntity<TSubclassOf<AActor>>(SpawnableEntities, InEntityName, bAllowEmpty);
}

bool ASimulationState::ServerSetEntityStateCheckRequest(FROSSetEntityState_Request Request) {
    if (PreviousSetEntityStateRequest.state_name == Request.state_name &&
        PreviousSetEntityStateRequest.state_reference_frame == Request.state_reference_frame &&
        PreviousSetEntityStateRequest.state_pose_position_x == Request.state_pose_position_x &&
        PreviousSetEntityStateRequest.state_pose_position_y == Request.state_pose_position_y &&
        PreviousSetEntityStateRequest.state_pose_position_z == Request.state_pose_position_z &&
        PreviousSetEntityStateRequest.state_pose_orientation == Request.state_pose_orientation
        ){
        return false;
    }
    else {
        return true;
    }
}

void ASimulationState::ServerSetEntityState(FROSSetEntityState_Request Request)
{
    if (ServerSetEntityStateCheckRequest(Request))
    {
        FVector pos(Request.state_pose_position_x, Request.state_pose_position_y, Request.state_pose_position_z);
        FTransform relativeTransf(Request.state_pose_orientation, pos);
        relativeTransf = ConversionUtils::TransformROSToUE(relativeTransf);
        FTransform worldTransf;
        URRGeneralUtils::GetWorldTransform(
            Request.state_reference_frame,
            Entities.Contains(Request.state_reference_frame) ? Entities[Request.state_reference_frame] : nullptr,
            relativeTransf,
            worldTransf);
        Entities[Request.state_name]->SetActorTransform(worldTransf);
    }

    PreviousSetEntityStateRequest = Request;
}


bool ASimulationState::ServerAttachCheckRequest(FROSAttach_Request Request) {
    if (PreviousAttachRequest.name1 == Request.name1 &&
        PreviousAttachRequest.name2 == Request.name2 ) {
        return false;
    }
    else {
        return true;
    }
}
void ASimulationState::ServerAttach(FROSAttach_Request Request)
{
    if (ServerAttachCheckRequest(Request))
    {
        AActor* Entity1 = Entities[Request.name1];
        AActor* Entity2 = Entities[Request.name2];

        if (!Entity2->IsAttachedTo(Entity1))
        {
            Entity2->AttachToActor(Entity1, FAttachmentTransformRules::KeepWorldTransform);
        }
        else
        {
            Entity2->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        }
    }

    PreviousAttachRequest = Request;
}

bool ASimulationState::ServerSpawnCheckRequest(FROSSpawnEntityRequest Request) {
    if (PreviousSpawnRequest.Xml == Request.Xml &&
            PreviousSpawnRequest.StateName == Request.StateName &&
            PreviousSpawnRequest.Xml == Request.Xml &&
            PreviousSpawnRequest.StatePosePositionX == Request.StatePosePositionX &&
            PreviousSpawnRequest.StatePosePositionY == Request.StatePosePositionY &&
            PreviousSpawnRequest.StatePosePositionZ == Request.StatePosePositionZ &&
            PreviousSpawnRequest.StatePoseOrientation == Request.StatePoseOrientation &&
            PreviousSpawnRequest.StateReferenceFrame == Request.StateReferenceFrame
    ){
        return false;
    }
    else {
        return true;
    }
}

void ASimulationState::ServerSpawnEntity(FROSSpawnEntityRequest Request)
{
    if(ServerSpawnCheckRequest(Request)) {
        const FString &entityModelName = Request.Xml;
        const FString &entityName = Request.StateName;

        FVector relLocation(Request.StatePosePositionX, Request.StatePosePositionY, Request.StatePosePositionZ);
        FTransform relativeTransf = ConversionUtils::TransformROSToUE(
                FTransform(Request.StatePoseOrientation, relLocation));
        FTransform worldTransf;
        URRGeneralUtils::GetWorldTransform(
                Request.StateReferenceFrame,
                Entities.Contains(Request.StateReferenceFrame)
                ? Entities[Request.StateReferenceFrame] : nullptr,
                relativeTransf,
                worldTransf);
        UE_LOG(LogRapyutaCore, Warning, TEXT("Spawning Entity of model [%s] as [%s]"), *entityModelName, *entityName);

        // TODO: details rationale to justify using SpawnActorDeferred
        AActor *newEntity = GetWorld()->SpawnActorDeferred<AActor>(SpawnableEntities[entityModelName],
                                                                   worldTransf);
        UROS2Spawnable *SpawnableComponent = NewObject<UROS2Spawnable>(newEntity, TEXT("ROS2 Spawn Parameters"));

        SpawnableComponent->RegisterComponent();
        SpawnableComponent->InitializeParameters(Request);
        SpawnableComponent->SetIsReplicated(true);

        newEntity->AddInstanceComponent(SpawnableComponent);
        newEntity->Rename(*entityName);
        newEntity->SetReplicates(true);
        newEntity->bAlwaysRelevant = true; //Needs to be set to relevant otherwise it won't consistantly replicate
#if WITH_EDITOR
        newEntity->SetActorLabel(*entityName);
#endif
//            UE_LOG(LogRapyutaCore, Warning, TEXT("request %s"), *Request.Tags);
        for (auto &tag: Request.Tags) {
            UE_LOG(LogRapyutaCore, Warning, TEXT("tag from request %s"), *tag);
            newEntity->Tags.Emplace(MoveTemp(tag));
            SpawnableComponent->AddTag(tag);
        }

        UGameplayStatics::FinishSpawningActor(newEntity, worldTransf);
        AddEntity(newEntity);
    }
    PreviousSpawnRequest = Request;
}

bool ASimulationState::ServerDeleteCheckRequest(FROSDeleteEntity_Request Request) {
    if (PreviousDeleteRequest.name == Request.name)  {
        return false;
    }
    else {
        return true;
    }
}

void ASimulationState::ServerDeleteEntity(FROSDeleteEntity_Request Request)
{
    if (ServerDeleteCheckRequest(Request))
    {
        AActor* Removed = Entities.FindAndRemoveChecked(Request.name);
        Removed->Destroy();
    }
    PreviousDeleteRequest = Request;
}
