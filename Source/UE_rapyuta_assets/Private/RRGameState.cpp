// Copyright (C) Rapyuta Robotics
#include "RRGameState.h"
// UE
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "GenericPlatform/GenericPlatformMath.h"
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
}

// Eg: FVector(50.f, -800.f, ARRHuman::HUMAN_SIZE_HALF_HEIGHT), FVector(2550.f, -1550.f, ARRHuman::HUMAN_SIZE_HALF_HEIGHT)
void ARRGameState::SpawnHumansAtRandomLocations(const FVector& InLocationA, const FVector& InLocationB)
{
    verify(FGenericPlatformMath::Min(InLocationA.Z, InLocationB.Z) >= ARRHuman::HUMAN_SIZE_HALF_HEIGHT);

    const auto fGetRandomLocation = [&InLocationA, &InLocationB]()
    { return URRMathUtils::GetRandomLocation(InLocationA, InLocationB); };

    TArray<FRRHumanInfo> humanGroupInfo;
    humanGroupInfo.SetNum(HUMAN_DEFAULT_NUM);
    for (auto& humanInfo : humanGroupInfo)
    {
        humanInfo.SpawnTransform = FTransform(fGetRandomLocation());
        humanInfo.MovementDestinations.Emplace(fGetRandomLocation());
        humanInfo.MovementDestinations.Emplace(fGetRandomLocation());
    }

    SpawnHumans(humanGroupInfo);
}

void ARRGameState::SpawnHumans(const TArray<FRRHumanInfo>& InHumanGroupInfo)
{
    for (const auto& humanInfo : InHumanGroupInfo)
    {
        auto* newHuman = Cast<ARRHuman>(UAIBlueprintHelperLibrary::SpawnAIFromClass(this,
                                                                                    HumanClass,
                                                                                    /*UBehaviorTree**/ nullptr,
                                                                                    humanInfo.SpawnTransform.GetTranslation(),
                                                                                    humanInfo.SpawnTransform.Rotator(),
                                                                                    true,
                                                                                    this));
        if (newHuman)
        {
            newHuman->SetupMovementPlan(humanInfo.MovementDestinations);
            HumanGroup.Add(newHuman);
        }
    }
}
