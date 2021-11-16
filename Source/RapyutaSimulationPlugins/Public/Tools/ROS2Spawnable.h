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

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ActorName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ActorNamespace;

public:

    UFUNCTION(BlueprintCallable)
    virtual void InitializeParameters(FROSSpawnEntity_Request Request);

    UFUNCTION(BlueprintCallable)
    virtual void SetName(FString Name);

    UFUNCTION(BlueprintCallable)
    virtual void SetNamespace(FString Namespace);

    UFUNCTION(BlueprintCallable)
    virtual FString GetName();

    UFUNCTION(BlueprintCallable)
    virtual FString GetNamespace();

};
