/**
 * @file RRROS2HitSensorComponent.h
 * @brief EntityState sensor components which publish entitystate relative to a specific actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "EngineUtils.h"

// rclUE
#include "RRROS2BaseSensorComponent.h"

#include <Msgs/ROS2HitEvent.h>

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"

#include "RRROS2HitSensorComponent.generated.h"

/**
 * @brief Entity(Actor) state publisher class
 * @todo add ROSService to change reference frame
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2HitEventPublisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    URRROS2HitEventPublisher()
    {
        TopicName = TEXT("entity_state");
        MsgClass = UROS2HitEventMsg::StaticClass();
        PublicationFrequencyHz = 0;    //Event based trigger
    };
};

/**
 * @brief Publish Status of  hit
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2HitSensorComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRROS2HitSensorComponent object
     *
     */
    URRROS2HitSensorComponent()
    {
        SensorPublisherClass = URRROS2HitEventPublisher::StaticClass();
    }

    void BeginPlay() override;

    /**
     * @brief Calculate relative pose with #URRGeneralUtils and update #Data
     * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
     */
    virtual void SensorUpdate() override;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UObject* TargetObject = nullptr;

    UFUNCTION(BlueprintCallable)
    void BoundCallbacks(UObject* InTargetObject);

    virtual void OnComponentHit(UPrimitiveComponent* HitComp,
                                AActor* OtherActor,
                                UPrimitiveComponent* OtherComp,
                                FVector NormalImpulse,
                                const FHitResult& Hit);

    virtual void OnActorHit(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& Hit);

    // ROS
    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UPROPERTY(BlueprintReadWrite)
    FROSHitEvent Data;

private:
    //     UPROPERTY()
    //     FTransform RootOffset = FTransform::Identity;
};
