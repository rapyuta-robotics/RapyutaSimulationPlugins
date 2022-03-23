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
    void Init();

    UFUNCTION(BlueprintCallable)
    void AddEntity(AActor* Entity);
//
//    UFUNCTION(BlueprintCallable)
//    void AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities);

    UFUNCTION(BlueprintCallable)
    TMap<FString, AActor*> GetEntity();

//    UFUNCTION(BlueprintCallable)
//    TMap<FString, TSubclassOf<AActor>> GetSpawnableEntities();
//
//    UFUNCTION(BlueprintCallable)
//    bool RequestInSpawnableEntities(FString RequestName);

//    UFUNCTION(BlueprintCallable)
//    bool IsValidRequestSpawnableEntities(FString RequestName);
//
//    UFUNCTION(BlueprintCallable)
//    TSubclassOf<AActor>* GetSpawnableEntity(FString RequestName);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    UPROPERTY()
    TMap<FName, FActors> EntitiesWithTag;
//    UPROPERTY(EditAnywhere, BlueprintReadWrite)
//    TMap<FString, TSubclassOf<AActor>> SpawnableEntities;

    // need to keep track of "Entities"? or just use a search?
};
