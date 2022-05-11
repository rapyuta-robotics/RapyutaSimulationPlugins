// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "EngineUtils.h"

// rclUE
#include "RRROS2BaseSensorComponent.h"

#include <Msgs/ROS2EntityStateMsg.h>

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"
#include "Tools/RRROS2EntityStatePublisher.h"

#include "RRROS2EntityStateSensorComponent.generated.h"

/**
 *
 */

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2EntityStateSensorComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    URRROS2EntityStateSensorComponent();

    void BeginPlay() override;

    virtual void SensorUpdate() override;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FString ReferenceActorName = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    AActor* ReferenceActor = nullptr;

    UFUNCTION(BlueprintCallable)
    virtual void SetReferenceActorByName(const FString& InName);

    UFUNCTION(BlueprintCallable)
    virtual void SetReferenceActorByActor(AActor* InActor);

    // ROS
    UFUNCTION(BlueprintCallable)
    virtual FROSEntityState GetROS2Data();

    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UPROPERTY(BlueprintReadWrite)
    FROSEntityState Data;
};
