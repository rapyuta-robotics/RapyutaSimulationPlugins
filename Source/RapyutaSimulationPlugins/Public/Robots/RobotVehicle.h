// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Drives/RobotVehicleMovementComponent.h"
#include "GameFramework/Pawn.h"

#include "RobotVehicle.generated.h"

class URobotVehicleMovementComponent;
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public APawn
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URobotVehicleMovementComponent* RobotVehicleMoveComponent = nullptr;

    virtual void InitializeMoveComponent();

    UFUNCTION(BlueprintCallable)
    virtual void SetLinearVel(const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable)
    virtual void SetAngularVel(const FVector& InAngularVelocity);

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;
};
