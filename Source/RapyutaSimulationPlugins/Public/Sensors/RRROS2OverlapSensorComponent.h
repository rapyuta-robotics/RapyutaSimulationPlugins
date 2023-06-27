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

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"

#include "RRROS2OverlapSensorComponent.generated.h"

/**
 * @brief Publish Status of  Overlap
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
    URRROS2OverlapSensorComponent()
    {
        // ROS 2 param
        TopicName = TEXT("overlaps");
        MsgClass = UROS2OverlapEventMsg::StaticClass();
        PublicationFrequencyHz = 0;    //Event based trigger
    }

    void BeginPlay() override;

    /**
     * @brief Calculate relative pose with #URRGeneralUtils and update #Data
     * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
     */
    virtual void SensorUpdate() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<UObject*> TargetObjects;

    UFUNCTION(BlueprintCallable)
    void BoundCallbacks(const TArray<UObject*> InTargetObjects);

    UFUNCTION()
    void OnOverlap(AActor* OverlappedActor,
                   AActor* OtherActor,
                   UPrimitiveComponent* OtherComp,
                   const bool InBegin,
                   const FString& Name = TEXT(""));

    UFUNCTION()
    void OnComponentOverlap(UPrimitiveComponent* OverlappedComponent,
                            AActor* OtherActor,
                            UPrimitiveComponent* OtherComp,
                            int32 OtherBodyIndex,
                            const bool InBegin);

    UFUNCTION()
    virtual void OnComponentBeginOverlap(UPrimitiveComponent* OverlappedComponent,
                                         AActor* OtherActor,
                                         UPrimitiveComponent* OtherComp,
                                         int32 OtherBodyIndex,
                                         bool bFromSweep,
                                         const FHitResult& SweepResult);

    UFUNCTION()
    virtual void OnComponentEndOverlap(UPrimitiveComponent* OverlappedComponent,
                                       AActor* OtherActor,
                                       UPrimitiveComponent* OtherComp,
                                       int32 OtherBodyIndex);

    UFUNCTION()
    void OnActorBeginOverlap(AActor* OverlappedActor, AActor* OtherActor);

    UFUNCTION()
    void OnActorEndOverlap(AActor* OverlappedActor, AActor* OtherActor);

    // ROS
    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UPROPERTY(BlueprintReadWrite)
    FROSOverlapEvent Data;

    // Ignore collision with Owner Actor or self
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIgnoreSelf = true;

    // List of object which collise with are ignored.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<UObject*> IgnoreList;

    bool IsIgnore(AActor* SelfActor, AActor* OtherActor, UPrimitiveComponent* OtherComp);
};
