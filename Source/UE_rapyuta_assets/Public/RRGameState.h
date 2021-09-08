// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "GameFramework/GameStateBase.h"

#include "RRGameState.generated.h"

class APawn;
class ARRPlayerController;
UCLASS(Config = RRSimSettings)
class UE_RAPYUTA_ASSETS_API ARRGameState : public AGameStateBase
{
    GENERATED_BODY()

public:
    ARRGameState();

    static constexpr int8 HUMAN_DEFAULT_NUM = 10;

    virtual void BeginPlay() override;
    UPROPERTY()
    ARRPlayerController* PlayerController = nullptr;

    UPROPERTY()
    TSubclassOf<ACharacter> HumanClass;

    UPROPERTY()
    TArray<APawn*> HumanGroup;

    UFUNCTION(BlueprintCallable)
    void SpawnHumansAtRandomLocations(const FVector& InLocationA, const FVector& InLocationB);

    UFUNCTION(BlueprintCallable)
    void SpawnHumans(const TArray<FRRHumanInfo>& InHumanGroupInfo);
};
