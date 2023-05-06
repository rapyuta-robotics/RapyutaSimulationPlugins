/**
 * @file ROS2Spawnable.h
 * @brief BaseComponents which is used when spawning Actor from ROS 2 service in #ASimulationState.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "Srvs/ROS2SpawnEntity.h"

#include "ROS2Spawnable.generated.h"

/**
 * @brief BaseComponents which is used when spawning Actor from ROS 2 service in #ASimulationState.
 * Set Actor Name and ROS 2 namespace of the ROS2Node with SpawnEntity srv.
 * @sa [ue_msgs/SpawnEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API UROS2Spawnable : public UActorComponent
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString ActorModelName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString ActorName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    FString ActorNamespace;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    TArray<FString> ActorTags;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Replicated)
    FString ActorJsonConfigs;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Replicated)
    FString ActorReferenceFrame;
    
    /**
     * @brief Set Actor name and ROS 2 namespace from SpawnEntity service request.
     * @sa [ue_msgs/SpawnEntity.srv](https://github.com/rapyuta-robotics/UE_msgs/blob/devel/srv/SpawnEntity.srv)
     * @param InRequest
     */
    UFUNCTION(BlueprintCallable)
    virtual void InitializeParameters(const FROSSpawnEntityReq& InRequest);

    UFUNCTION(BlueprintCallable)
    void SetActorModelName(const FString& InModelName);

    UFUNCTION(BlueprintCallable)
    virtual void SetName(const FString& InName);

    UFUNCTION(BlueprintCallable)
    virtual void SetNamespace(const FString& InNamespace);

    UFUNCTION(BlueprintCallable)
    virtual void AddTag(const FString& InTag);

    UFUNCTION(BlueprintCallable)
    virtual FString GetName() const;

    UFUNCTION(BlueprintCallable)
    virtual FString GetNamespace() const;

    UFUNCTION(BlueprintCallable)
    virtual int32 GetNetworkPlayerId() const;

    UFUNCTION(BlueprintCallable)
    virtual void SetNetworkPlayerId(const int32 InNetworkPlayerId);

protected:
    virtual void OnComponentCreated() override;

    //! Player ID No. This is used to check which network client posses this Robot.
    UPROPERTY(Replicated)
    int32 NetworkPlayerId;
};
