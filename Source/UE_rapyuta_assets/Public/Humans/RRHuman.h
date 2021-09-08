// Copyright (C) Rapyuta Robotics
#pragma once

// UE
#include "AITypes.h"
#include "GameFramework/Character.h"
#include "Navigation/PathFollowingComponent.h"

#include "RRHuman.generated.h"

USTRUCT(Blueprintable)
struct UE_RAPYUTA_ASSETS_API FRRHumanInfo
{
    GENERATED_BODY()

    UPROPERTY()
    FTransform SpawnTransform = FTransform::Identity;

    UPROPERTY()
    TArray<FVector> MovementDestinations;
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class UE_RAPYUTA_ASSETS_API ARRHuman : public ACharacter
{
    GENERATED_BODY()

public:
    ARRHuman();
    static constexpr float HUMAN_SIZE_RADIUS = 42.f;
    static constexpr float HUMAN_SIZE_HALF_HEIGHT = 96.f;

    virtual void SetupMovementPlan(const TArray<FVector>& InMovementDestinations);

    UFUNCTION()
    void MoveToNextDestination(FAIRequestID RequestID, EPathFollowingResult::Type MovementResult);
    // This is the new recommended api, but does not support UFUNCTION() specifier yet.
    // void MoveToNextDestination(FAIRequestID RequestID, const FPathFollowingResult& Result);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FVector> MovementDestinations;

    UPROPERTY(VisibleAnywhere)
    int32 CurrentDestinationIdx = 0;
};
