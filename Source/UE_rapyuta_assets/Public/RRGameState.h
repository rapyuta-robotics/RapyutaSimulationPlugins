// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "GameFramework/GameStateBase.h"

#include "RRGameState.generated.h"

class ARRHuman;
class ARRPlayerController;
UCLASS(Config = RRSimSettings)
class UE_RAPYUTA_ASSETS_API ARRGameState : public AGameStateBase
{
    GENERATED_BODY()

public:
    ARRGameState();

    static constexpr int32 HUMAN_NUM = 100;
    static constexpr float HUMAN_YAW = 360.f / HUMAN_NUM;

    virtual void BeginPlay() override;
    UPROPERTY()
    ARRPlayerController* PlayerController = nullptr;

    UPROPERTY()
    TSubclassOf<ACharacter> HumanClass;

    UPROPERTY()
    TArray<ARRHuman*> HumanGroup;

    UFUNCTION()
    void SpawnHumans();
};
