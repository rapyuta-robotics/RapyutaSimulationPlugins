// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Tools/RRGeneralUtils.h"
#include "Tools/UEUtilities.h"

#include "SimulationState.generated.h"

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FActorsWithTag
{
    GENERATED_BODY()

    UPROPERTY()
    FName tag;

    UPROPERTY()
    TArray<AActor*> Actors;
};

class UROS2GenericSrv;

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ASimulationState : public AActor
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    ASimulationState();

public:
    UFUNCTION(BlueprintCallable)
    void Init(AROS2Node* InROS2Node);

    UFUNCTION(BlueprintCallable)
    void GetEntityStateSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void SetEntityStateSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void AttachSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void SpawnEntitySrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void DeleteEntitySrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void AddEntity(AActor* Entity);

    UFUNCTION(BlueprintCallable)
    void AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities);

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
    TMap<FName, FActorsWithTag> EntitiesWithTag;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, TSubclassOf<AActor>> SpawnableEntities;

    // need to keep track of "Entities"? or just use a search?
};
