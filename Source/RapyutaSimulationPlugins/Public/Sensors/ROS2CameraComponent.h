// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

#include <Msgs/ROS2ImageMsg.h>
#include "ROS2BaseSensorComponent.h"
#include "Tools/ROS2ImagePublisher.h"

#include "ROS2CameraComponent.generated.h"

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
class RAPYUTASIMULATIONPLUGINS_API UROS2CameraComponent : public UROS2BaseSensorComponent
{
	GENERATED_BODY()
	
public:

	UROS2CameraComponent();
	
	virtual void CreatePublisher(const FString& InPublisherName = TEXT("")) override;

    virtual void PreInitializePublisher(AROS2Node* InROS2Node, const FString& InTopicName) override;

    virtual void Run() override;
	
protected:
	
	UFUNCTION()
	void MessageUpdate(UROS2GenericMsg *TopicMessage);

	void CaptureNonBlocking();

	void TakeImage();

	TQueue<FRenderRequest*> RenderRequestQueue;

	FROSImage Data;

	UPROPERTY()
    FTimerHandle TimerHandle;

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
	int32 FPS = 30;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 Width = 640;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 Height = 480;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 QueueSize = 2;

	// ROS 
	UFUNCTION(BlueprintCallable)
	virtual FROSImage GetROS2Data();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString Encoding = TEXT("rgb8");;
};
