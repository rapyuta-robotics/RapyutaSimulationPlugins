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

#include "SimulationStateData.generated.h"

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FActors
        {
                GENERATED_BODY()

                UPROPERTY()
                TArray<AActor*> Actors;
        };

class UROS2GenericSrv;

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ASimulationStateData : public AActor
{
    GENERATED_BODY()

public:
    ASimulationStateData();

    UFUNCTION(BlueprintCallable)
    void AddEntity(AActor* Entity);

    UFUNCTION(BlueprintCallable)
    void OnRep_Entity();

    UFUNCTION(BlueprintCallable)
    void AddTaggedEntities(AActor* Entity, const FName& InTag);

    UFUNCTION(BlueprintCallable)
    void AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, ReplicatedUsing = OnRep_Entity)
    TArray<AActor*> EntityList;

    UPROPERTY()
    TMap<FName, FActors> EntitiesWithTag;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, TSubclassOf<AActor>> SpawnableEntities;

    // need to keep track of "Entities"? or just use a search?
};
