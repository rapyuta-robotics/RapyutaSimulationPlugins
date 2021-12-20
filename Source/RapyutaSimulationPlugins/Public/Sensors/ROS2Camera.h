// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "ROS2Node.h"
#include <Msgs/ROS2ImageMsg.h>
#include "ROS2Publisher.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"


#include "ROS2Camera.generated.h"

/**
 * 
 */
USTRUCT()
struct FRenderRequest{
	GENERATED_BODY()
	TArray<FColor> Image;
	FRenderCommandFence RenderFence;
};

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API AROS2Camera : public AROS2Node
{
	GENERATED_BODY()
	
public:

	AROS2Camera();

	virtual void Init();
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	UFUNCTION()
	void MessageUpdate(UROS2GenericMsg *TopicMessage);

	void CaptureNonBlocking();

	TQueue<FRenderRequest*> RenderRequestQueue;

	FROSImage Data;

public:

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	USceneCaptureComponent2D *SceneCaptureComponent = nullptr;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	UTextureRenderTarget2D *RenderTarget = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UROS2Publisher* Publisher;

	UFUNCTION(BlueprintCallable)
	virtual FROSImage GetData();
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString TopicName = TEXT("img_raw");

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 PublishFreq = 1;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int Width = 640;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int Height = 480;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int FOV = 90;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int OrthWidth = 320;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString Encoding = TEXT("rgb8");;
};
