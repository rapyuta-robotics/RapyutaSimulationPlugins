/**
 * @file TurtlebotBurger.h
 * @brief Example of child class of #ARRBaseRobot
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Drives/RRPhysicsJointComponent.h"
#include "Robots/Turtlebot3/TurtlebotBurgerBase.h"

#include "RRTurtlebotBurger.generated.h"

/**
 * @brief Example of child class of #ARRBaseRobot
 * Uses #UDifferentialDriveComponent and has #URR2DLidarComponent.
 * This class is designed to be inheritted from Blueprint class to be assigned UStaticMeshComponent.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRTurtlebotBurger : public ATurtlebotBurgerBase
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new ARRTurtlebotBurger object. Calls #SetupBody
    *
    * @param ObjectInitializer
    */
    ARRTurtlebotBurger(const FObjectInitializer& ObjectInitializer);

protected:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRPhysicsJointComponent* Base_WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRPhysicsJointComponent* Base_WheelRight = nullptr;

    /**
     * @brief Create UStaticMeshComponent, create UPhysicsConstraintComponent.
     *
     */
    bool SetupBody() override;

    /**
     * @brief Setup material, relative location, anugular/linear limits, drive params.
     *
     */
    bool SetupConstraintsAndPhysics() override;

    /**
     * @brief Setup #UDifferentialDriveComponent
     *
     */
    void SetupWheelDrives() override;
};
