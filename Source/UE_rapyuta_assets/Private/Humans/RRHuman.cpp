// Copyright (C) Rapyuta Robotics
#include "Humans/RRHuman.h"

#include "AIController.h"
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "Components/CapsuleComponent.h"

#if !WITH_EDITOR
#include "DetourCrowdAIController.h"
#endif

ARRHuman::ARRHuman()
{
#if !WITH_EDITOR
    // (snote) [ADetourCrowdAIController] seems to be an Engine-only component, thus could only specified through a child BP class
    verify(AIControllerClass == ADetourCrowdAIController::StaticClass());
#endif

    // Set size for player capsule
    UCapsuleComponent* CapsuleComponent = GetCapsuleComponent();
    if (CapsuleComponent)
    {
        CapsuleComponent->InitCapsuleSize(HUMAN_SIZE_RADIUS, HUMAN_SIZE_HEIGHT);
    }
}

void ARRHuman::SetupMovementPlan()
{
    // (snote) [ReceiveMoveCompleted] is a dynamic delegate, which does not support [AddLambda] yet :(
    UAIBlueprintHelperLibrary::GetAIController((AActor*)this)->ReceiveMoveCompleted.AddDynamic(this, &ARRHuman::SwapDestinations);
}

void ARRHuman::SwapDestinations(FAIRequestID RequestID, EPathFollowingResult::Type MovementResult)
{
    Swap(DestinationA, DestinationB);
    UAIBlueprintHelperLibrary::GetAIController((AActor*)this)->MoveToLocation(DestinationA);
}
