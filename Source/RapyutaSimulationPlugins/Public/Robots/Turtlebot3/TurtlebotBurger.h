/**
 * @file TurtlebotBurger.h
 * @brief Example of child class of #ARRBaseRobot
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

// RapyutaSimulationPlugins
#include "Robots/Turtlebot3/TurtlebotBurgerBase.h"

#include "TurtlebotBurger.generated.h"

/**
 * @brief Example of child class of #ARRBaseRobot
 * Uses #UDifferentialDriveComponent
 * This class is designed to be inheritted from Blueprint class to be assigned UStaticMeshComponent.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotBurger : public ATurtlebotBurgerBase
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new ATurtlebotBurger object. Calls #SetupBody
    *
    * @param ObjectInitializer
    */
    ATurtlebotBurger(const FObjectInitializer& ObjectInitializer);

protected:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_WheelRight = nullptr;

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
