/**
 * @file RRROS2ActorTFPublisher.h
 * @brief Publish TF of Actor relative to the reference actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "Kismet/GameplayStatics.h"
#include "Math/TransformNonVectorized.h"

// rclUE
#include "Msgs/ROS2GenericMsg.h"
#include "Msgs/ROS2TFMsg.h"
#include "Srvs/ROS2SetBool.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2TFPublisher.h"

#include "RRROS2ActorTFPublisher.generated.h"

/**
 * @brief Publish TF of Actor relative to the reference actor.
 * If reference actor name is empty, publishes TF from world origin.
 * Provides ROS2Service to start/stop publishing tf.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2ActorTFPublisher : public URRROS2TFPublisher

{
    GENERATED_BODY()

public:
    bool InitializeWithROS2(UROS2NodeComponent* InROS2Node) override;

    /**
     * @brief Callback function of TriggerPublishSrv
     * @sa [example_interfaces/SetBool.srv](https://github.com/ros2/example_interfaces/blob/master/srv/SetBool.srv)
     */
    UFUNCTION(BlueprintCallable)
    void TriggerPublishSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    virtual void SetReferenceActorByName(const FString& InName);

    UFUNCTION(BlueprintCallable)
    virtual void SetReferenceActorByActor(AActor* InActor);

    UFUNCTION(BlueprintCallable)
    virtual void SetTargetActorByName(const FString& InName);

    UFUNCTION(BlueprintCallable)
    virtual void SetTargetActorByActor(AActor* InActor);

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    AActor* ReferenceActor = nullptr;

    //! if this value is empty, publish pose from world origin.
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FString ReferenceActorName = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    AActor* TargetActor = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FString TargetActorName = TEXT("");

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bIsValid = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FString TriggerServiceName = TEXT("actor_tf_publisher_trigger");

    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
