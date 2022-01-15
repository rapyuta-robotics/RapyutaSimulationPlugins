// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

#include "ROS2Node.h"
#include <Msgs/ROS2ImageMsg.h>
#include "ROS2Publisher.h"

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
class RAPYUTASIMULATIONPLUGINS_API UROS2CameraComponent : public UCameraComponent
{
	GENERATED_BODY()
	
public:

	UROS2CameraComponent();

	UFUNCTION(BlueprintCallable)
	virtual void Init();
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bAutoStart = true;

	// ROS 
	UFUNCTION(BlueprintCallable)
	virtual FROSImage GetData();

    UPROPERTY(Transient)
    AROS2Node* Node;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UROS2Publisher* Publisher;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString NodeName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString Namespace;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString TopicName = TEXT("img_raw");

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 PublishFreq = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString FrameId = TEXT("camera");;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString Encoding = TEXT("rgb8");;
};
