// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "Kismet/GameplayStatics.h"
#include "Math/TransformNonVectorized.h"

// rclUE
#include "Msgs/ROS2GenericMsg.h"
#include "Msgs/ROS2TFMsg.h"
#include "Srvs/ROS2SetBoolSrv.h"
#include "ROS2Publisher.h"

#include "RRROS2ActorTFPublisher.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2ActorTFPublisher : public URRROS2TFPublisher
{
    GENERATED_BODY()

public:

	void BeginPlay() override;
    
    void InitializeWithROS2(AROS2Node* InROS2Node) override;

    UFUNCTION(BlueprintCallable)
    void TriggerPublishSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
	virtual void SetReferenceActor(const FString& InName);

    UFUNCTION(BlueprintCallable)
	virtual void SetTargetActor(const FString& InName);

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
	AActor* ReferenceActor = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
	FString ReferenceActorName = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
	AActor* TargetActor = nullptr;
    
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
	FString TargetActorName = TEXT("");

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	bool bIsValid = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
	FString TriggerServiceName = TEXT("actor_tf_publisher_trigger");

    void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
