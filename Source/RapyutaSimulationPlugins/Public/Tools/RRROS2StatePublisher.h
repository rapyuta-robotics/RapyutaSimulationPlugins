// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2EntityStateMsg.h"
#include "ROS2Publisher.h"

#include "RRROS2StatePublisher.generated.h"

class AROS2Node;

/**
 * This class should be attached to a robot and publish its states
 * It is responsibility of the owning robot to update the structs, this way the class can be kept more flexible and work with robots
 * defined by multiple actors as well as robots defined by a skeletal mesh This could use a refactor once the robots are more well
 * defined and could require a refactor of the main publisher class as well, to avoid the iterator Idx as it is used now Ideally, it
 * should be this class that fetches all the necessary data to be published
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2StatePublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    void InitializeWithROS2(AROS2Node* InROS2Node) override;
    void UpdateMessage(UROS2GenericMsg* InMessage) override;

    UFUNCTION(BlueprintCallable)
    void AddEntityToPublish(const FString& InName,
                            const FVector& InPosition,
                            const FRotator& InOrientation,
                            const FString& InRefFrame);

    // it's responsibility of the owner to update this
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FROSEntityState> StatesToPublish;

    UPROPERTY()
    int32 Idx = 0;
};
