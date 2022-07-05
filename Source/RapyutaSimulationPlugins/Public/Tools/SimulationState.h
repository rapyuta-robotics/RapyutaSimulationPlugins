// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"
#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

#include "Srvs/ROS2AttachSrv.h"
#include "Srvs/ROS2DeleteEntitySrv.h"
#include "Srvs/ROS2GetEntityStateSrv.h"
#include "Srvs/ROS2SetEntityStateSrv.h"
#include "Srvs/ROS2SpawnEntitySrv.h"
#include "SimulationState.generated.h"

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FActors
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<AActor*> Actors;
};

class UROS2GenericSrv;

// (NOTE) To be renamed ARRROS2SimulationState, due to its inherent attachment to ROS2 Node
// & thus house [Entities] spawned by ROS services, and  with ROS relevance.
// However, check for its usage in BP and refactor if there is accordingly!
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ASimulationState : public AActor
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    ASimulationState();

public:
    UFUNCTION(BlueprintCallable)
    virtual void Init(AROS2Node* InROS2Node);

    UFUNCTION(BlueprintCallable)
    virtual void InitEntities();

    UFUNCTION(BlueprintCallable)
    virtual void InitROS2Node(AROS2Node* InROS2Node);

    //Get Entity Functions
//    UFUNCTION(BlueprintCallable)
//    bool ServerGetEntityStateCheckRequest(FROSGetEntityState_Request Request);
//
//    UFUNCTION(BlueprintCallable)
//    void ServerGetEntityState(FROSGetEntityState_Request Request);

    UPROPERTY(BlueprintReadOnly)
    FROSGetEntityState_Request PreviousGetEntityStateRequest;

    //Set Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerSetEntityStateCheckRequest(FROSSetEntityState_Request Request);

    UFUNCTION(BlueprintCallable)
    void ServerSetEntityState(FROSSetEntityState_Request Request);

    UPROPERTY(BlueprintReadOnly)
    FROSSetEntityState_Request PreviousSetEntityStateRequest;


    //Attach Functions
    UFUNCTION(BlueprintCallable)
    bool ServerAttachCheckRequest(FROSAttach_Request Request);

    UFUNCTION(BlueprintCallable)
    void ServerAttach(FROSAttach_Request Request);

    UPROPERTY(BlueprintReadOnly)
    FROSAttach_Request PreviousAttachRequest;

    //Spawn Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerSpawnCheckRequest(FROSSpawnEntityRequest Request);

    UFUNCTION(BlueprintCallable)
    void ServerSpawnEntity(FROSSpawnEntityRequest Request);

    UPROPERTY(BlueprintReadOnly)
    FROSSpawnEntityRequest PreviousSpawnRequest;

    //Delete Entity Functions
    UFUNCTION(BlueprintCallable)
    bool ServerDeleteCheckRequest(FROSDeleteEntity_Request Request);

    UFUNCTION(BlueprintCallable)
    void ServerDeleteEntity(FROSDeleteEntity_Request Request);

    UPROPERTY(BlueprintReadOnly)
    FROSDeleteEntity_Request PreviousDeleteRequest;

    //Simulation State Manipulation Functions
    UFUNCTION(BlueprintCallable)
    void AddEntity(AActor* Entity);

    UFUNCTION(BlueprintCallable)
    void OnRep_Entity();

    UFUNCTION(BlueprintCallable)
    void OnRep_SpawnableEntity();

    UFUNCTION(BlueprintCallable)
    void AddTaggedEntities(AActor* Entity, const FName& InTag);

    // BP callable thus the param could not be const&
    UFUNCTION(BlueprintCallable)
    void AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities);

    UFUNCTION(BlueprintCallable)
    void GetSplitSpawnableEntities();

    template<typename T>
    bool CheckEntity(TMap<FString, T>& InEntities, const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckEntity(const FString& InEntityName, const bool bAllowEmpty = false);
    bool CheckSpawnableEntity(const FString& InEntityName, const bool bAllowEmpty = false);

    // need node that will handle services - this class will only define and register the service
    UPROPERTY(BlueprintReadOnly)
    AROS2Node* ROSServiceNode = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    UPROPERTY()
    TMap<FName, FActors> EntitiesWithTag;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_Entity)
    TArray<AActor*> EntityList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_SpawnableEntity)
    TArray<TSubclassOf<AActor>> SpawnableEntityList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_SpawnableEntity)
    TArray<FString> SpawnableEntityNameList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, TSubclassOf<AActor>> SpawnableEntities;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle TimerHandle;
    // need to keep track of "Entities"? or just use a search?
};
