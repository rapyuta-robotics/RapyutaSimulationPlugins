// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "Srvs/ROS2SpawnEntitySrv.h"

#include "ROS2Spawnable.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UROS2Spawnable : public UActorComponent
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString ActorName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString ActorNamespace;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TArray<FString> ActorTags;

public:
    UFUNCTION(BlueprintCallable)
    virtual void InitializeParameters(const FROSSpawnEntityRequest& InRequest);

    UFUNCTION(BlueprintCallable)
    virtual void SetName(const FString& InName);

    UFUNCTION(BlueprintCallable)
    virtual void SetNamespace(const FString& InNamespace);

    UFUNCTION(BlueprintCallable)
    virtual void AddTag(const FString& InTag);

    UFUNCTION(BlueprintCallable)
    virtual FString GetName();

    UFUNCTION(BlueprintCallable)
    virtual FString GetNamespace();

};
