// Copyright (C) Rapyuta Robotics
#include "RRGameState.h"
// UE
#include "Kismet/GameplayStatics.h"
#include "UObject/ConstructorHelpers.h"

// RRSim
#include "Humans/RRHuman.h"
#include "RRMathUtils.h"
#include "RRPlayerController.h"

ARRGameState::ARRGameState()
{
    HumanClass =
        ConstructorHelpers::FObjectFinderOptional<UClass>(TEXT("Class'/UE_rapyuta_Assets/Blueprints/BPHuman.BPHuman_C'")).Get();
    verify(HumanClass);
}

void ARRGameState::BeginPlay()
{
    Super::BeginPlay();

    PlayerController = Cast<ARRPlayerController>(UGameplayStatics::GetPlayerController(this, 0));
    verify(PlayerController);

    SpawnHumans();
}

void ARRGameState::SpawnHumans()
{
    const FVector centerLocation = FVector(1250.f, -800.f, ARRHuman::HUMAN_SIZE_HALF_HEIGHT);

    static constexpr float HUMAN_GROUP_CIRCLE_RADIUS = 500.f;
    for (auto i = 0; i < HUMAN_NUM; ++i)
    {
        FRotator spawnRotation = FRotator(0.f, HUMAN_YAW * i, 0.f);
        const FVector spawnLocation =
            centerLocation + HUMAN_GROUP_CIRCLE_RADIUS * spawnRotation.RotateVector(FVector::ForwardVector);
        auto* newHuman = Cast<ARRHuman>(UAIBlueprintHelperLibrary::SpawnAIFromClass(
            this, HumanClass, nullptr, spawnLocation, FRotator::ZeroRotator /*FRotator(0.f, HUMAN_YAW * (i + 2), 0.f)*/, true));
        if (newHuman)
        {
            newHuman->DestinationA = spawnLocation;
            newHuman->DestinationB =
                spawnLocation + FVector(URRMathUtils::FRandRange(-HUMAN_GROUP_CIRCLE_RADIUS, HUMAN_GROUP_CIRCLE_RADIUS),
                                        URRMathUtils::FRandRange(-HUMAN_GROUP_CIRCLE_RADIUS, HUMAN_GROUP_CIRCLE_RADIUS),
                                        0.f);
            newHuman->SetupMovementPlan();
            HumanGroup.Add(newHuman);
        }
    }
}
