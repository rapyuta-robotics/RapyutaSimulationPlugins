/**
 * @file RRROS2StatePublisher.h
 * @brief This class should be attached to a robot and publish its states.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2EntityState.h"
#include "ROS2Node.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"

#include "RRROS2StatePublisher.generated.h"

/**
 * @brief This class should be attached to a robot and publish its states.
 * It is responsibility of the owning robot to update the structs, this way the class can be kept more flexible and work with robots
 * defined by multiple actors as well as robots defined by a skeletal mesh. This could use a refactor once the robots are more well
 * defined and could require a refactor of the main publisher class as well, to avoid the iterator Idx as it is used now Ideally, it
 * should be this class that fetches all the necessary data to be published.
 * @todo Implementation should follow the other publisher classes.
 * @sa [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2StatePublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:
    URRROS2StatePublisher();
    
    void UpdateMessage(UROS2GenericMsg* InMessage) override;

    UFUNCTION(BlueprintCallable)
    void AddEntityToPublish(const FString& InName,
                            const FVector& InPosition,
                            const FRotator& InOrientation,
                            const FString& InRefFrame);

    //! it's responsibility of the owner to update this
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FROSEntityState> StatesToPublish;

    UPROPERTY()
    int32 Idx = 0;

    UPROPERTY()
    TWeakObjectPtr<ARobotVehicle> Robot = nullptr;
    virtual void SetTargetRobot(ARobotVehicle* InRobot);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ReferenceFrameId;
};
