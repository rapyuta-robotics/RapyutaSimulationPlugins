// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#include "Core/RRCharacter.h"

// RapyutaSimulationPlugins
#include "Core/RRCrowdAIController.h"

ARRCharacter::ARRCharacter()
{
    SetupDefaultCharacter();
}

ARRCharacter::ARRCharacter(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefaultCharacter();
}

void ARRCharacter::SetupDefaultCharacter()
{
    AIControllerClass = ARRCrowdAIController::StaticClass();
    AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;

    // To make the character rotate toward the destination only
    bUseControllerRotationYaw = false;
    bUseControllerRotationPitch = false;
    bUseControllerRotationRoll = false;
}

void ARRCharacter::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    InitCharacterMovementComp();
    // NOTE: In child classes, here configure SkeletalMesh by GetMesh()->SetSkeletalMesh()
}

void ARRCharacter::InitCharacterMovementComp()
{
    // Configure character movement, created in parent class' ctor
    UCharacterMovementComponent* characterMovement = GetCharacterMovement();
    check(characterMovement);
    characterMovement->bUseControllerDesiredRotation = false;
    characterMovement->bUseSeparateBrakingFriction = false;
    // NOTE: Rotate character to moving direction (make sure bUseControllerRotationYaw as false for it)
    characterMovement->bOrientRotationToMovement = true;
    characterMovement->RotationRate = FRotator(0.f, 640.f, 0.f);
}
