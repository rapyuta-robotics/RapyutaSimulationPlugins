// Copyright (C) Rapyuta Robotics
#pragma once
#include "GameFramework/PlayerController.h"
#include "UE_rapyuta_assets.h"

#include "RRPlayerController.generated.h"

#define RAPYUTA_PAWN_MOUSE_FOLLOWING (1)

class APawn;
class ARRGameState;
UCLASS()
class UE_RAPYUTA_ASSETS_API ARRPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ARRPlayerController();

    UPROPERTY()
    uint32 bMovePawnsToMouseCursor : 1;

    UPROPERTY()
    ARRGameState* GameState = nullptr;

public:
    virtual void BeginPlay() override;

    virtual void PlayerTick(float DeltaTime) override;
    virtual void SetupInputComponent() override;

    /** Navigate player to the current mouse cursor location. */
    UFUNCTION()
    void MovePawnsToMouseCursor();

    /** Navigate player to the given world location. */
    UFUNCTION()
    void SetPawnsNewDestination(const TArray<APawn*> InPawnGroup, const FVector& DestLocation);

    /** Input handlers for SetDestination action. */
    UFUNCTION()
    void OnSetPawnsDestinationPressed();
    UFUNCTION()
    void OnSetPawnsDestinationReleased();
};
