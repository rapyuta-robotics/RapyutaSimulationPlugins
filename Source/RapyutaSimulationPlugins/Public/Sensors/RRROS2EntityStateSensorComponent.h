/**
 * @file RRROS2EntityStateSensorComponent.h
 * @brief EntityState sensor components which publish entitystate relative to a specific actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "EngineUtils.h"

// rclUE
#include "RRROS2BaseSensorComponent.h"

#include <Msgs/ROS2EntityState.h>

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"

#include "RRROS2EntityStateSensorComponent.generated.h"

DECLARE_MULTICAST_DELEGATE_OneParam(FOnNewReferenceActorDetected, AActor* /* NewReferenceActor */);

/**
 * @brief EntityState sensor components which publish entitystate relative to a specific actor.
 * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2EntityStateSensorComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRROS2EntityStateSensorComponent object
     *
     */
    URRROS2EntityStateSensorComponent();

    void BeginPlay() override;

    /**
     * @brief Calculate relative pose with #URRGeneralUtils and update #Data
     * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
     */
    virtual void SensorUpdate() override;

    //! NOTE: Only #URRPoseSensorManager uses #ReferenceActor
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FString ReferenceActorName = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    AActor* ReferenceActor = nullptr;
    FOnNewReferenceActorDetected OnNewReferenceActorDetected;

    UFUNCTION(BlueprintCallable)
    virtual void SetReferenceActorByName(const FString& InName);

    UFUNCTION(BlueprintCallable)
    virtual void SetRootOffset(const FTransform& InRootOffset);

    UFUNCTION(BlueprintCallable)
    virtual void SetReferenceActorByActor(AActor* InActor);

    // ROS
    /**
     * @brief return #Data
     *
     * @return FROSEntityState
     */
    UFUNCTION(BlueprintCallable)
    virtual FROSEntityState GetROS2Data();

    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UPROPERTY(BlueprintReadWrite)
    FROSEntityState Data;

private:
    UPROPERTY()
    FTransform RootOffset = FTransform::Identity;
};
