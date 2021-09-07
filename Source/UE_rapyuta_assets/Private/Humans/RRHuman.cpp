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

void ARRHuman::SetupMovementPlan()
{
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

#if !RAPYUTA_HUMAN_MOUSE_FOLLOWING
    // [ReceiveMoveCompleted] is a dynamic delegate, which does not support [AddLambda] yet :(
    UAIBlueprintHelperLibrary::GetAIController((AActor*)this)->ReceiveMoveCompleted.AddDynamic(this, &ARRHuman::SwapDestinations);
#endif
}

void ARRHuman::SwapDestinations(FAIRequestID RequestID, EPathFollowingResult::Type MovementResult)
{
    Swap(DestinationA, DestinationB);
    UAIBlueprintHelperLibrary::GetAIController((AActor*)this)->MoveToLocation(DestinationA);
}
