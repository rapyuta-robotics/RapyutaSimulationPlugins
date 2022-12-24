/**
 * @file RRROS2TFPublisher.h
 * @brief TF publisher class.
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
#include "ROS2Publisher.h"

#include "RRROS2TFPublisher.generated.h"

/**
 * @brief TF Publisher class. Please check #URRROS2OdomPublisher as example.
 * @sa [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URRROS2TFPublisher object
    *
    */
    URRROS2TFPublisher();

    //! Publish static tf or not. @sa https://docs.ros.org/en/rolling/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html?highlight=static%20tf
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool IsStatic = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId;

    //! @sa https://docs.unrealengine.com/5.1/en-US/API/Runtime/Core/Math/FTransform/
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform TF = FTransform::Identity;

    /**
     * @brief Initialize publisher with QoS
     *
     * @param InROS2Node
     */
    void InitializeWithROS2(AROS2Node* InROS2Node) override;

    /**
     * @brief Initialize.
     * @todo Is this method necessary?
     * @param InROS2Node
     */
    UFUNCTION(BlueprintCallable)
    void InitTFPublisher(AROS2Node* InROS2Node)
    {
        InitializeWithROS2(InROS2Node);
    }

    /**
     * @brief Set value to #TF.
     *
     * @param Translation
     * @param Rotation
     */
    UFUNCTION(BlueprintCallable)
    void SetTransform(const FVector& Translation, const FQuat& Rotation);

    /**
     * @brief Update message frorm #TF.
     *
     * @param InMessage
     */
    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
