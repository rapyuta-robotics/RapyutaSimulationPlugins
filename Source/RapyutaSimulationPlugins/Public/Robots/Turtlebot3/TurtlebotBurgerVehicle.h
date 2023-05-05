/**
 * @file TurtlebotBurgerVehicle.h
 * @brief Kinematic robot example of #ARRBaseRobot.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/SkeletalMeshComponent.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Robots/RRBaseRobot.h"
#include "Sensors/RR2DLidarComponent.h"

#include "TurtlebotBurgerVehicle.generated.h"

/**
 * @brief Kinematic robot example of #ARRBaseRobot.
 * Robot with Skeletal Mesh component and 2D Lidar.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotBurgerVehicle : public ARRBaseRobot
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ATurtlebotBurgerVehicle object
     *
     */
    ATurtlebotBurgerVehicle();

    /**
     * @brief Construct a new ATurtlebotBurgerVehicle object
     *
     * @param ObjectInitializer
     */
    ATurtlebotBurgerVehicle(const FObjectInitializer& ObjectInitializer);

    //! Robot Mesh
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Replicated)
    USkeletalMeshComponent* SkeletalMeshComp = nullptr;

    /**
     * @brief Initialize #SkeletalMeshComp.
     *
     */
    void SetupDefaultRootSkeletal();

    /**
     * @brief Returns the properties used for network replication, this needs to be overridden by all actor classes with native
     * replicated properties
     *
     * @param OutLifetimeProps Output lifetime properties
     */
    virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URR2DLidarComponent* LidarComponent = nullptr;

protected:
    /**
     * @brief Pre-Initialize components
     */
    void PreInitializeComponents() override;
};
