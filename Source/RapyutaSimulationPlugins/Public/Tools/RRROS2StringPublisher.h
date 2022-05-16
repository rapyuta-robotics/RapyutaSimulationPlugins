/**
 * @file RRROS2StringPublisher.h
 * @brief String publisher class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */


#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "ROS2Publisher.h"

#include "RRROS2StringPublisher.generated.h"

class AROS2Node;

/**
 * @brief String Publisher class
 * @sa [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2StringPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URRROS2StringPublisher object
    * 
    */
    URRROS2StringPublisher();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Message;

    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
