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
    static ConstructorHelpers::FObjectFinder<UClass> sHumanBPClass(TEXT("/UE_rapyuta_Assets/Blueprints/BPHuman.BPHuman"));
    HumanClass = sHumanBPClass.Object;
    check(HumanClass);
}

void ARRGameState::BeginPlay()
{
    Super::BeginPlay();
    SpawnHumans();

    PlayerController = Cast<ARRPlayerController>(UGameplayStatics::GetPlayerController(this, 0));
}

void ARRGameState::SpawnHumans()
{
    FVector playerLocation;
    FRotator playerRotation;
    PlayerController->GetPlayerViewPoint(playerLocation, playerRotation);
    const FVector centerLocation = FVector(playerLocation.X, playerLocation.Y, 200.f);

    static constexpr float HUMAN_GROUP_CIRCLE_RADIUS = 2000.f;
    for (auto i = 0; i < HUMAN_NUM; ++i)
    {
        FRotator rotation = FRotator(0.f, HUMAN_YAW * i, 0.f);
        const FVector location = centerLocation + HUMAN_GROUP_CIRCLE_RADIUS * rotation.RotateVector(FVector::ForwardVector);
        auto* newHuman = Cast<ARRHuman>(UAIBlueprintHelperLibrary::SpawnAIFromClass(
            this, HumanClass, nullptr, location, FRotator(0.f, HUMAN_YAW * i + 180.f, 0.f), true));
        if (newHuman)
        {
            newHuman->DestinationA = location;
            newHuman->DestinationB =
                centerLocation + FVector(URRMathUtils::FRandRange(-HUMAN_GROUP_CIRCLE_RADIUS, HUMAN_GROUP_CIRCLE_RADIUS),
                                         URRMathUtils::FRandRange(-HUMAN_GROUP_CIRCLE_RADIUS, HUMAN_GROUP_CIRCLE_RADIUS),
                                         0.f);
            newHuman->SetupMovementPlan();
            HumanGroup.Add(newHuman);
        }
    }
}
