// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameState.h"
#include "GameFramework/Actor.h"

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
class RAPYUTASIMULATIONPLUGINS_API ASimulationStateData : public AGameState
{
    GENERATED_BODY()

//public:
//    // Sets default values for this actor's properties
//    ASimulationStateData();

public:
    UFUNCTION(BlueprintCallable)
    void AddEntity(AActor* Entity);

    UFUNCTION(BlueprintCallable)
    void AddSpawnedEntity(AActor* Entity);

    UFUNCTION(BlueprintCallable)
    TMap<FString, AActor*> GetEntity();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TArray<AActor*>  EntityList;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TArray<AActor*>  SpawnedEntityList;

    UPROPERTY()
    TMap<FName, FActors> EntitiesWithTag;

};
