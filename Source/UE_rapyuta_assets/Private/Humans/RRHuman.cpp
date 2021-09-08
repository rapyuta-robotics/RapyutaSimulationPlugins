// Copyright (C) Rapyuta Robotics
#include "Humans/RRHuman.h"

// UE
#include "AIController.h"
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/CharacterMovementComponent.h"

// RRSim
#include "RRPlayerController.h"
#if !WITH_EDITOR
#include "DetourCrowdAIController.h"
#endif

ARRHuman::ARRHuman()
{
#if !WITH_EDITOR
    // [ADetourCrowdAIController] seems to be an Engine-only component, thus could only specified through a child BP class
    verify(AIControllerClass == ADetourCrowdAIController::StaticClass());
#endif

    // Set size for player capsule
    UCapsuleComponent* CapsuleComponent = GetCapsuleComponent();
    if (CapsuleComponent)
    {
        CapsuleComponent->InitCapsuleSize(HUMAN_SIZE_RADIUS, HUMAN_SIZE_HALF_HEIGHT);
    }
}

void ARRHuman::SetupMovementPlan(const TArray<FVector>& InMovementDestinations)
{
    verify(InMovementDestinations.Num() > 0);
    MovementDestinations = InMovementDestinations;
    CurrentDestinationIdx = 0;

    // To make the human rotate toward the destination only
    bUseControllerRotationYaw = false;
    bUseControllerRotationPitch = false;
    bUseControllerRotationRoll = false;

    // Configure character movement
    UCharacterMovementComponent* characterMovement = Cast<UCharacterMovementComponent>(GetMovementComponent());
    if (characterMovement)
    {
        characterMovement->bUseControllerDesiredRotation = false;
        characterMovement->bUseSeparateBrakingFriction = false;
        // Rotate character to moving direction
        characterMovement->bOrientRotationToMovement = true;
        characterMovement->RotationRate = FRotator(0.f, 640.f, 0.f);
    }

    // [ReceiveMoveCompleted] is a dynamic delegate, which does not support [AddLambda] yet :(
    UAIBlueprintHelperLibrary::GetAIController(this)->ReceiveMoveCompleted.AddDynamic(this, &ARRHuman::MoveToNextDestination);
}

void ARRHuman::MoveToNextDestination(FAIRequestID RequestID, EPathFollowingResult::Type MovementResult)
{
    if (++CurrentDestinationIdx == MovementDestinations.Num())
    {
        CurrentDestinationIdx = 0;
    }
    UAIBlueprintHelperLibrary::GetAIController(this)->MoveToLocation(MovementDestinations[CurrentDestinationIdx]);
}
