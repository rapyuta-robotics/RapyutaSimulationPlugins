/**
 * @file RRPoseSensorManager.h
 * @brief Robot pose sensor manager. If #RefActorSelectMode=AUTO, RefActor is automatically updated with nearlest actor with Tag
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Sensors/RRROS2EntityStateSensorComponent.h"
#include "Tools/SimulationState.h"

#include "RRPoseSensorManager.generated.h"

/**
 * @brief Reference actor selection mode. AUTO means it is automatically updated to the nearlest actor along Z axis with Tag
 */
UENUM(BlueprintType)
enum class ERRRefActorSelectMode : uint8
{
    MANUAL UMETA(DisplayName = "Manual"),
    AUTO UMETA(DisplayName = "Auto")
};

/**
 * @brief Robot pose sensor manager which has two #URRROS2EntityStateSensorComponent which publishes
 * - Reference Actor pose from world origin
 * - Pose from reference actor.
 * Typical usecase is to set map origin as reference actor and publish robot pose from map_origin. 
 * This is useful to navigate robot in the large area such as multiple floor building which has multiple map origins.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRPoseSensorManager : public URRROS2EntityStateSensorComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRPoseSensorManager object, configuring sensor default property values
     */
    URRPoseSensorManager();

    //! Handle to server's main sim state
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Replicated)
    ASimulationState* ServerSimState = nullptr;

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    /**
     * @brief Create and initialize publisher and start sensor update by calling
     * #CreatePublisher, #PreInitializePublisher, #InitializePublisher and #Run.
     *
     * @param InROS2Node ROS2Node which this publisher belongs to
     * @param InPublisherName Publisher component name
     * @param InTopicName Topic name
     * @param InQoS Topic QoS
     *
     * @sa [AROS2Node](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dcb/class_a_r_o_s2_node.html)
     * @sa [ROS2 QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
     */
    virtual void InitalizeWithROS2(UROS2NodeComponent* InROS2Node,
                                   const FString& InPublisherName = EMPTY_STR,
                                   const FString& InTopicName = EMPTY_STR,
                                   const TEnumAsByte<UROS2QoS> InQoS = UROS2QoS::SensorData) override;

    /**
     * @brief Calculate relative pose with #URRGeneralUtils and update #Data
     * @todo Currently twist = ZeroVectors. Should be filled for physics actors.
     */
    virtual void SensorUpdate() override;

    /**
     * @brief Update reference actor to the nearest one along Z axis with tag
     */
    UFUNCTION(BlueprintCallable)
    virtual void UpdateReferenceActorWithTag();

    //! Reference actor's tag
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString ReferenceTag = URRActorCommon::MAP_ORIGIN_TAG;

    //! Reference actor's selection mode
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    ERRRefActorSelectMode RefActorSelectMode = ERRRefActorSelectMode::AUTO;

    //! Map frame id
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString MapFrameId = URRActorCommon::MAP_ROS_FRAME_ID;

    //! Map origin's pose sensor
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    URRROS2EntityStateSensorComponent* MapOriginPoseSensor = nullptr;

protected:
    /**
     * @brief Callback on component creation to setup SetIsReplicated()
     */
    virtual void OnComponentCreated() override;
};
