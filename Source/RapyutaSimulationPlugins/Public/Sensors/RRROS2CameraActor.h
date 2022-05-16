/**
 * @file RRROS2CameraActor.h
 * @brief Standalone Camera actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Sensors/RRROS2CameraComponent.h"

// rclUE
#include "ROS2Node.h"

#include "RRROS2CameraActor.generated.h"

/**
 * @brief Standalone camera actor which can be placed in the level with #URRROS2CameraComponent.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRROS2CameraActor : public AActor
{
	GENERATED_BODY()
	
public:
	/**
	 * @brief Construct a new ARRROS2CameraActor object
	 * 
	 */
	ARRROS2CameraActor();

    UPROPERTY(Transient)
    AROS2Node* Node;

    UPROPERTY(BlueprintReadWrite)
    FString NodeName;

    UPROPERTY(BlueprintReadWrite)
    FString NodeNamespace;

protected:
	/**
	 * @brief Initialize ROS2 Node and #CameraComponent
	 * If NodeName is empty, node name become This actor name + _RRROS2CameraNode
	 */
	virtual void BeginPlay() override;
	
	//! ROS2 camera component for this camera
	UPROPERTY(Category = CameraActor, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	URRROS2CameraComponent* CameraComponent;
};
