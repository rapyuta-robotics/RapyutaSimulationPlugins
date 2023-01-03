/**
 * @file RRROS2OdomPublisher.h
 * @brief Odometry Topic and TF publisher of #ARRRobotBaseVehicle
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2Odom.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2TFPublisher.h"

#include "RRROS2OdomPublisher.generated.h"

class UROS2GenericMsg;
class ARRRobotBaseVehicle;

/**
 * @brief Odometry Topic and TF publisher of #ARRRobotBaseVehicle
 * @sa [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2OdomPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRROS2OdomPublisher object
     *
     */
    URRROS2OdomPublisher();

    UPROPERTY(BlueprintReadWrite)
    TWeakObjectPtr<ARRRobotBaseVehicle> RobotVehicle = nullptr;

    UPROPERTY(BlueprintReadWrite)
    URRROS2TFPublisher* TFPublisher = nullptr;

    void InitializeTFWithROS2(UROS2NodeComponent* InROS2Node);

    bool InitializeWithROS2(UROS2NodeComponent* InROS2Node) override;

    /**
     * @brief Initialize odom publisher
     * @todo is this method necessary?
     */
    UFUNCTION(BlueprintCallable)
    void InitOdomPublisher(UROS2NodeComponent* InROS2Node)
    {
        InitializeWithROS2(InROS2Node);
    }

    // void RevokeUpdateCallback() override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
    bool GetOdomData(FROSOdom& OutOdomData) const;

    //! Publish tf or not
    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdomTf = false;

    //! add robot name to the frame_id and ChildFrameId or not.
    UPROPERTY(BlueprintReadWrite)
    bool bAppendNodeNamespace = true;
};
