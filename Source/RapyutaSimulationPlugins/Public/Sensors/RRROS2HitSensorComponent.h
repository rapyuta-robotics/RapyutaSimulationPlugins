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
 * @brief Publish Hit status with OnActorHit and OnComponentHit
 * Publish event when hit event happened from #TargetObjects
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
        // ROS 2 param
        TopicName = TEXT("collisions");
        MsgClass = UROS2HitEventMsg::StaticClass();
        PublicationFrequencyHz = 0;    //Event based trigger
    }

    void BeginPlay() override;

    //! List of UObject which is whether child class of PrimitiveComponent or Actor
    //! Topic is published when Hit event happen from the object in this list.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<UObject*> TargetObjects;

    //! Bind OnActorHit and OnComponentHit delegate with #TargetObjects
    UFUNCTION(BlueprintCallable)
    virtual void BindCallback(UObject* InTargetObject);

    //! Common hit process called from #OnTargetComponentHit and #OnTargetActorHit
    //! Fill data and publish topic.
    UFUNCTION()
    virtual void OnHit(AActor* SelfActor,
                       AActor* OtherActor,
                       FVector NormalImpulse,
                       const FHitResult& Hit,
                       const FString& Name = TEXT(""));

    UFUNCTION()
    virtual void OnTargetComponentHit(UPrimitiveComponent* HitComp,
                                      AActor* OtherActor,
                                      UPrimitiveComponent* OtherComp,
                                      FVector NormalImpulse,
                                      const FHitResult& Hit);

    UFUNCTION()
    virtual void OnTargetActorHit(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& Hit);

    // ROS
    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UPROPERTY(BlueprintReadWrite)
    FROSHitEvent Data;

    //! Ignore collision with Owner Actor or self component.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIgnoreSelf = true;

    //! List of object which collide with are ignored.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<UObject*> IgnoreList;

    bool IsIgnore(AActor* SelfActor, AActor* OtherActor, UPrimitiveComponent* OtherComp);
};
