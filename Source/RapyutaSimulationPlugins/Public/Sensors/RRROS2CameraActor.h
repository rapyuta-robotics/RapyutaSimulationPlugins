// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Sensors/RRROS2CameraComponent.h"

// rclUE
#include "ROS2Node.h"

#include "RRROS2CameraActor.generated.h"

/**
 * 
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRROS2CameraActor : public AActor
{
	GENERATED_BODY()
	
public:

	ARRROS2CameraActor();

    UPROPERTY(Transient)
    AROS2Node* Node;

    UPROPERTY(BlueprintReadWrite)
    FString NodeName;

    UPROPERTY(BlueprintReadWrite)
    FString NodeNamespace;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	/** The camera component for this camera */
	UPROPERTY(Category = CameraActor, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	URRROS2CameraComponent* CameraComponent;
};
