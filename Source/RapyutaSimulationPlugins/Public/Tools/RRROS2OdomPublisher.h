/**
 * @file RRROS2OdomPublisher.h
 * @brief Odometry Topic and TF publisher of #ARobotVehicle
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2OdometryMsg.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2TFPublisher.h"

#include "RRROS2OdomPublisher.generated.h"

class UROS2GenericMsg;
class ARobotEmptyVehicle;

/**
 * @brief Odometry Topic and TF publisher of #ARobotEmptyVehicle
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
    TWeakObjectPtr<ARobotEmptyVehicle> RobotVehicle = nullptr;

    UPROPERTY(BlueprintReadWrite)
    URRROS2TFPublisher* TFPublisher = nullptr;

    void InitializeTFWithROS2(AROS2Node* InROS2Node);

    void InitializeWithROS2(AROS2Node* InROS2Node) override;

    /**
     * @brief Initialize odom publisher
     * @todo is this method necessary?
     */
    UFUNCTION(BlueprintCallable)
    void InitOdomPublisher(AROS2Node* InROS2Node)
    {
        InitializeWithROS2(InROS2Node);
    }

    void RevokeUpdateCallback() override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
    bool GetOdomData(FROSOdometry& OutOdomData) const;

    //! Publish tf or not
    UPROPERTY(BlueprintReadWrite)
    bool bPublishOdomTf = false;

    //! add robot name to the frame_id and child_frame_id or not.
    UPROPERTY(BlueprintReadWrite)
    bool bAppendNodeNamespace = true;
};
