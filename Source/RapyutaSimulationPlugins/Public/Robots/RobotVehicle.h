/**
 * @file RobotVehicle.h
 * @brief Base RobotVehicle class.
 * Example is #ATurtlebotBurger.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Robots/RRRobotBaseVehicle.h"

// rclUE
#include "ROS2Node.h"

#include "RobotVehicle.generated.h"

/**
 * @brief RobotVehicle class.
 * This class represents robot vehicles built up from a Skeletal Mesh component that is also its Root.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARobotVehicle : public ARRRobotBaseVehicle
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ARobotVehicle object
     *
     */
    ARobotVehicle();

    /**
     * @brief Construct a new ARobotVehicle object
     *
     * @param ObjectInitializer
     */
    ARobotVehicle(const FObjectInitializer& ObjectInitializer);

    //! Robot Mesh
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    /**
     * @brief Initialize #SkeletalMeshComp.
     *
     */
    void SetupDefaultRootSkeletal();

    UFUNCTION(BlueprintCallable, Server, Reliable)
    virtual void SetServerLinearVel(float TimeStamp, const FVector& InPosition, const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable, Server, Reliable)
    virtual void SetServerAngularVel(float TimeStamp,const FRotator& InRotation, const FVector& InAngularVelocity);

    UFUNCTION(BlueprintCallable, Client, Reliable)
    virtual void SetClientLinearVel(const FVector& InLinearVelocity);

    UFUNCTION(BlueprintCallable, Client, Reliable)
    virtual void SetClientAngularVel(const FVector& InAngularVelocity);
};
