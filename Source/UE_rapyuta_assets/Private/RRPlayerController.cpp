// Copyright (C) Rapyuta Robotics
#include "RRPlayerController.h"

// UE
#include "AIController.h"
#include "Async/ParallelFor.h"
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "NavigationSystem.h"

// RRSim
#include "RRGameState.h"

#define RAPYUTA_HUMAN_MOUSE_FOLLOWING (0)

ARRPlayerController::ARRPlayerController()
{
#if RAPYUTA_HUMAN_MOUSE_FOLLOWING
    bShowMouseCursor = true;
#endif
    DefaultMouseCursor = EMouseCursor::Crosshairs;
}

void ARRPlayerController::BeginPlay()
{
    Super::BeginPlay();

    GameState = Cast<ARRGameState>(UGameplayStatics::GetGameState(GetWorld()));
    check(GameState);
    SetHumansNewDestination(FVector::ZeroVector);
}

void ARRPlayerController::PlayerTick(float DeltaTime)
{
    Super::PlayerTick(DeltaTime);

#if RAPYUTA_HUMAN_MOUSE_FOLLOWING
    if (bMoveToMouseCursor)
    {
        MoveToMouseCursor();
    }
#endif
}

void ARRPlayerController::SetupInputComponent()
{
    Super::SetupInputComponent();

    InputComponent->BindAction("SetDestination", IE_Pressed, this, &ARRPlayerController::OnSetDestinationPressed);
    InputComponent->BindAction("SetDestination", IE_Released, this, &ARRPlayerController::OnSetDestinationReleased);
}

void ARRPlayerController::MoveToMouseCursor()
{
    // Trace to see what is under the mouse cursor
    FHitResult Hit;
    GetHitResultUnderCursor(ECC_Visibility, false, Hit);

    if (Hit.bBlockingHit)
    {
        // We hit something, move there
        SetHumansNewDestination(Hit.ImpactPoint);
    }
}

void ARRPlayerController::SetHumansNewDestination(const FVector& InNewDestination)
{
    ParallelFor(GameState->HumanGroup.Num(),
                [this, InNewDestination](uint32 InHumanIndex)
                {
                    ARRHuman* human = GameState->HumanGroup[InHumanIndex];
                    UAIBlueprintHelperLibrary::GetAIController((AActor*)human)->MoveToLocation(InNewDestination);
                });
}

void ARRPlayerController::OnSetDestinationPressed()
{
    bMoveToMouseCursor = true;
}

void ARRPlayerController::OnSetDestinationReleased()
{
    bMoveToMouseCursor = false;
}
