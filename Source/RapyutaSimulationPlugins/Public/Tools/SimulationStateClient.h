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
#include "Srvs/ROS2AttachSrv.h"
#include "Srvs/ROS2DeleteEntitySrv.h"
#include "Srvs/ROS2GetEntityStateSrv.h"
#include "Srvs/ROS2SetEntityStateSrv.h"
#include "Srvs/ROS2SpawnEntitySrv.h"

#include "SimulationStateClient.generated.h"

//USTRUCT()
//struct RAPYUTASIMULATIONPLUGINS_API FActors
//{
//    GENERATED_BODY()
//
//    UPROPERTY()
//    TArray<AActor*> Actors;
//};

class UROS2GenericSrv;

// (NOTE) To be renamed ARRROS2SimulationState, due to its inherent attachment to ROS2 Node
// & thus house [Entities] spawned by ROS services, and  with ROS relevance.
// However, check for its usage in BP and refactor if there is accordingly!
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API USimulationStateClient : public UActorComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    USimulationStateClient();

public:
    UFUNCTION(BlueprintCallable)
    virtual void InitSimulationState();

    UFUNCTION(BlueprintCallable)
    virtual void InitROS2Node(AROS2Node* InROS2Node);

//Get Entity State Service
    UFUNCTION(BlueprintCallable)
    void GetEntityStateSrv(UROS2GenericSrv* Service);


//Set Entity State Service
    UFUNCTION(BlueprintCallable)
    void SetEntityStateSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerSetEntityState(FROSSetEntityState_Request Request);

//Attach Service
    UFUNCTION(BlueprintCallable)
    void AttachSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerAttach(FROSAttach_Request Request);

//Spawn Service
    UFUNCTION(BlueprintCallable)
    void SpawnEntitySrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerSpawnEntity(FROSSpawnEntityRequest Request);

//    UFUNCTION(BlueprintCallable)
//    void SpawnEntityCheck();
//
//    UFUNCTION(BlueprintCallable)
//    void SpawnEntityResponse();

//Delete Service
    UFUNCTION(BlueprintCallable)
    void DeleteEntitySrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable, Server, Reliable)
    void ServerDeleteEntity(FROSDeleteEntity_Request Request);

    // need node that will handle services - this class will only define and register the service
    UPROPERTY(BlueprintReadOnly)
    UROS2SpawnEntitySrv* SpawnEntityService = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    FROSSpawnEntityResponse SpawnResponse;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle TimerHandle;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle TimerHandleResponder;

    UPROPERTY(BlueprintReadOnly)
    AROS2Node* ROSServiceNode = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* SimulationState = nullptr;

    // need to keep track of "Entities"? or just use a search?
};
