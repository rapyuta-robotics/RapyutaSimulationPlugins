// Copyright (C) Rapyuta Robotics
#include "RRPlayerController.h"

// UE
#include "AIController.h"
#include "Async/ParallelFor.h"
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "GameFramework/Pawn.h"
#include "Kismet/GameplayStatics.h"
#include "NavigationSystem.h"

// RRSim
#include "Humans/RRHuman.h"
#include "RRGameState.h"

ARRPlayerController::ARRPlayerController()
{
    bShowMouseCursor = true;
    DefaultMouseCursor = EMouseCursor::Crosshairs;
}

void ARRPlayerController::BeginPlay()
{
    Super::BeginPlay();

    GameState = Cast<ARRGameState>(UGameplayStatics::GetGameState(GetWorld()));
    check(GameState);
}

void ARRPlayerController::PlayerTick(float DeltaTime)
{
    Super::PlayerTick(DeltaTime);

#if RAPYUTA_PAWN_MOUSE_FOLLOWING
    if (bMovePawnsToMouseCursor)
    {
        MovePawnsToMouseCursor();
    }
#endif
}

void ARRPlayerController::SetupInputComponent()
{
    Super::SetupInputComponent();

    InputComponent->BindAction("SetPawnsDestination", IE_Pressed, this, &ARRPlayerController::OnSetPawnsDestinationPressed);
    InputComponent->BindAction("SetPawnsDestination", IE_Released, this, &ARRPlayerController::OnSetPawnsDestinationReleased);
    verify(PlayerInput);
}

void ARRPlayerController::MovePawnsToMouseCursor()
{
    // Trace to see what is under the mouse cursor
    FHitResult Hit;
    GetHitResultUnderCursor(ECC_Visibility, false, Hit);

    if (Hit.bBlockingHit)
    {
        SetPawnsNewDestination(GameState->HumanGroup, Hit.ImpactPoint);
    }
}

void ARRPlayerController::SetPawnsNewDestination(const TArray<APawn*> InPawnGroup, const FVector& InNewDestination)
{
    ParallelFor(InPawnGroup.Num(),
                [this, &InPawnGroup, InNewDestination](uint32 InPawnIndex)
                {
                    APawn* pawn = InPawnGroup[InPawnIndex];
                    UAIBlueprintHelperLibrary::GetAIController(pawn)->MoveToLocation(InNewDestination);
                });
}

void ARRPlayerController::OnSetPawnsDestinationPressed()
{
    bMovePawnsToMouseCursor = true;
}

void ARRPlayerController::OnSetPawnsDestinationReleased()
{
    bMovePawnsToMouseCursor = false;
}
