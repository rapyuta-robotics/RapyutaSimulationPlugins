// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

#include <Msgs/ROS2ImageMsg.h>
#include "RRROS2BaseSensorComponent.h"
#include "Tools/RRROS2ImagePublisher.h"

#include "RRROS2CameraComponent.generated.h"

/**
 * 
 */
USTRUCT()
struct FRenderRequest{
	GENERATED_BODY()
	TArray<FColor> Image;
	FRenderCommandFence RenderFence;
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2CameraComponent : public URRROS2BaseSensorComponent
{
	GENERATED_BODY()
	
public:

	URRROS2CameraComponent();
	
    virtual void PreInitializePublisher(AROS2Node* InROS2Node, const FString& InTopicName) override;

    virtual void SensorUpdate() override;
	    
protected:
	
	UFUNCTION()

	void CaptureNonBlocking();

	// void TakeImage();

	TQueue<FRenderRequest*> RenderRequestQueue;

	FROSImage Data;

	int32 QueueCount = 0;

public:
	// Camera
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	UCameraComponent *CameraComponent = nullptr;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	USceneCaptureComponent2D *SceneCaptureComponent = nullptr;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	UTextureRenderTarget2D *RenderTarget = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 Width = 640;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 Height = 480;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 QueueSize = 2;

	// ROS 
	UFUNCTION(BlueprintCallable)
	virtual FROSImage GetROS2Data();

	virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString Encoding = TEXT("rgb8");;
};
