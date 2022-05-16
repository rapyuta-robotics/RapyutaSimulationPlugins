/**
 * @file ROS2Spawnable.h
 * @brief BaseComponents which is used when spawning Actor from ROS2 service in #ASimulationState.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */


#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "Srvs/ROS2SpawnEntitySrv.h"

#include "ROS2Spawnable.generated.h"

/**
 * @brief BaseComponents which is used when spawning Actor from ROS2 service in #ASimulationState.
 * Set Actor Name and ROS2 namespace of the ROS2Node with SpawnEntity srv.
 * @sa [ue_msgs/SpawnEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv)
 */
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
    /**
     * @brief Set Actor name and ROS2 namespace from SpawnEntity service request.
     * @sa [ue_msgs/SpawnEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv)
     * @param InRequest 
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitializeParameters(const FROSSpawnEntityRequest& InRequest);

    UFUNCTION(BlueprintCallable)
    virtual void SetName(const FString& InName);

    UFUNCTION(BlueprintCallable)
    virtual void SetNamespace(const FString& InNamespace);

    UFUNCTION(BlueprintCallable)
    virtual FString GetName();

    UFUNCTION(BlueprintCallable)
    virtual FString GetNamespace();
};
