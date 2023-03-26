/**
 * @file RRROS2OdomPublisher.h
 * @brief Odometry Topic and TF publisher of #ARRBaseRobot
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
#include "Tools/RRROS2BaseSensorPublisher.h"

#include "RRROS2OdomPublisher.generated.h"

class UROS2GenericMsg;
class ARRBaseRobot;

/**
 * @brief Odometry Topic and TF publisher of #ARRBaseRobot
 * @sa [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2OdomPublisher : public URRROS2BaseSensorPublisher
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRROS2OdomPublisher object
     *
     */
    URRROS2OdomPublisher();

    virtual bool InitializeWithROS2(UROS2NodeComponent* InROS2Node) override;

    UPROPERTY(BlueprintReadWrite)
    URRROS2TFPublisher* TFPublisher = nullptr;

    void InitializeTFWithROS2(UROS2NodeComponent* InROS2Node);

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
