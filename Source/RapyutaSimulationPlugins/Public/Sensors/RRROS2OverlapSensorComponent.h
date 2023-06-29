/**
 * @file RRROS2OverlapSensorComponent.h
 * @brief EntityState sensor components which publish entitystate relative to a specific actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "EngineUtils.h"

// rclUE
#include "RRROS2BaseSensorComponent.h"

#include <Msgs/ROS2OverlapEvent.h>
#include <Msgs/ROS2Overlaps.h>

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"

#include "RRROS2OverlapSensorComponent.generated.h"

/**
 * @brief Publish overlappping acotors and components and begin/end event.
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2OverlapSensorComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRROS2OverlapSensorComponent object
     *
     */
    URRROS2OverlapSensorComponent();

    /**
     * @brief Initialize #EventPublisher and #SensorPublisher
     *
     * @param InROS2Node
     * @param InPublisherName
     * @param InTopicName
     * @param InQoS
     */
    virtual void InitalizeWithROS2(UROS2NodeComponent* InROS2Node,
                                   const FString& InPublisherName = EMPTY_STR,
                                   const FString& InTopicName = EMPTY_STR,
                                   const UROS2QoS InQoS = UROS2QoS::SensorData) override;

    void BeginPlay() override;

    /**
     * @brief Update #Overlaps by GetOverlappingComponents and GetOverlappingActors of #TargetObjects
     */
    virtual void SensorUpdate() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<UObject*> TargetObjects;

    //! Add name of TargetObject to #Overlaps and #BindCallback
    void AddTarget(UObject* InTargetObject);

    //! Bind OnComponentBeginOverlap. OnComponentEndOverlap, OnActorBeginOverlap and OnActorEndOverlap
    void BindCallback(UObject* InTargetObject);

    //! Common overlap process called from #OnComponentOverlap, #OnTargetActorBeginOverlap and #OnTargetActorEndOverlap
    //! Fill data and publish topic.
    void OnOverlap(AActor* OverlappedActor,
                   AActor* OtherActor,
                   UPrimitiveComponent* OtherComp,
                   const bool InBegin,
                   const FString& Name = TEXT(""));

    //! Common component overlap process called from #OnTargetComponentBeginOverlap and #OnTargetComponentEndOverlap
    //! Fill data and call #OnOverlap
    void OnComponentOverlap(UPrimitiveComponent* OverlappedComponent,
                            AActor* OtherActor,
                            UPrimitiveComponent* OtherComp,
                            int32 OtherBodyIndex,
                            const bool InBegin);

    UFUNCTION()
    virtual void OnTargetComponentBeginOverlap(UPrimitiveComponent* OverlappedComponent,
                                               AActor* OtherActor,
                                               UPrimitiveComponent* OtherComp,
                                               int32 OtherBodyIndex,
                                               bool bFromSweep,
                                               const FHitResult& SweepResult);

    UFUNCTION()
    virtual void OnTargetComponentEndOverlap(UPrimitiveComponent* OverlappedComponent,
                                             AActor* OtherActor,
                                             UPrimitiveComponent* OtherComp,
                                             int32 OtherBodyIndex);

    UFUNCTION()
    void OnTargetActorBeginOverlap(AActor* OverlappedActor, AActor* OtherActor);

    UFUNCTION()
    void OnTargetActorEndOverlap(AActor* OverlappedActor, AActor* OtherActor);

    // ROS
    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    /**
     * @brief Publisher for begin/end overlapping event
     *
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2Publisher* EventPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString EventTopicName = TEXT("");

    //! Overlap Event data
    UPROPERTY(BlueprintReadWrite)
    FROSOverlapEvent Data;

    //! List of Overlapping Actors and Components
    UPROPERTY(BlueprintReadWrite)
    FROSOverlaps Overlaps;

    //! Ignore collision with Owner Actor or self component.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIgnoreSelf = true;

    //! List of object which collide with are ignored.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<UObject*> IgnoreList;

    bool IsIgnore(AActor* SelfActor, AActor* OtherActor, UPrimitiveComponent* OtherComp);
};
