/**
 * @file RRTurtlebotBurger.h
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
 * Uses #URRDifferentialDriveComponent.
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
     * @brief Create UStaticMeshComponent, create URRPhysicsJointComponent.
     *
     */
    bool SetupBody() override;

    /**
     * @brief Setup URRPhysicsJointComponent.
     *
     */
    bool SetupConstraintsAndPhysics() override;

    /**
     * @brief Setup #URRDifferentialDriveComponent
     *
     */
    void SetupWheelDrives() override;
};
