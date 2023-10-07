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

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFPublisherBase : public UROS2Publisher
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URRROS2TFPublisher object
    *
    */
    URRROS2TFPublisherBase();

    //! Publish static tf or not. @sa https://docs.ros.org/en/rolling/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html?highlight=static%20tf
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool IsStatic = false;

    /**
     * @brief Initialize publisher with QoS
     *
     * @param InROS2Node
     */
    bool InitializeWithROS2(UROS2NodeComponent* InROS2Node) override;

    UFUNCTION(BlueprintCallable)
    virtual void AddTFtoMsg(FROSTFMsg& tf, const FString InFrameId, const FString InChildFrameId, const FTransform& InTF);

    virtual void GetROS2Msg(FROSTFMsg& tf){};

    /**
     * @brief Update message frorm #TF.
     *
     * @param InMessage
     */
    virtual void UpdateMessage(UROS2GenericMsg* InMessage) override;
};

/**
 * @brief TF Publisher class. Please check #URRROS2OdomPublisher as example.
 * @sa [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFPublisher : public URRROS2TFPublisherBase
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform TF = FTransform::Identity;

    /**
     * @brief Set value to #TF.
     *
     * @param Translation
     * @param Rotation
     */
    UFUNCTION(BlueprintCallable)
    void SetTransform(const FVector& Translation, const FQuat& Rotation);

    virtual void GetROS2Msg(FROSTFMsg& tf) override;
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFComponent : public UObject
{
    GENERATED_BODY()

public:
    URRROS2TFComponent(){};

    void Init(FString InFrameId, FString InChildFrameId)
    {
        FrameId = InFrameId;
        ChildFrameId = InChildFrameId;
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform TF = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId;

    UFUNCTION(BlueprintCallable)
    virtual FTransform GetTF()
    {
        return TF;
    };
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFsPublisher : public URRROS2TFPublisherBase

{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<URRROS2TFComponent*> TFComponents;

    virtual void GetROS2Msg(FROSTFMsg& tf) override;
};
