// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

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
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool IsStatic = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform TF = FTransform::Identity;

    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    UFUNCTION(BlueprintCallable)
    void InitTFPublisher(AROS2Node* InROS2Node)
    {
        InitializeWithROS2(InROS2Node);
    }

    UFUNCTION(BlueprintCallable)
    void SetTransform(const FVector& Translation, const FQuat& Rotation);

    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
