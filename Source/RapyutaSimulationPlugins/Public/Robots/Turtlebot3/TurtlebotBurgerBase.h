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
#include "Robots/RRBaseRobot.h"
#include "Robots/Turtlebot3/RRTurtlebotROS2Interface.h"
#include "Sensors/RR2DLidarComponent.h"

#include "TurtlebotBurgerBase.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogTurtlebotBurger, Log, All);

/**
 * @brief Example of child class of #ARRBaseRobot and base class for TurtlebotBurger.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotBurgerBase : public ARRBaseRobot
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new ATurtlebotBurgerBase object. Calls #SetupBody
    *
    * @param ObjectInitializer
    */
    ATurtlebotBurgerBase(const FObjectInitializer& ObjectInitializer);

protected:
    /**
     * @brief calls #SetupWheelDrives
     *
     */
    virtual void PostInitializeComponents() override;

    /**
     * @brief Create UStaticMeshComponent, create UPhysicsConstraintComponent.
     *
     */
    UFUNCTION()
    virtual bool SetupBody();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* Base = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* LidarSensor = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URR2DLidarComponent* LidarComponent = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* WheelRight = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* CasterBack = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_LidarSensor = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_CasterBack = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxForce = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UMaterial* VehicleMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UMaterial* BallMaterial = nullptr;

    UPROPERTY(VisibleAnywhere)
    uint8 bBodyComponentsCreated : 1;

    /**
     * @brief Setup UPhysicsConstraintComponent.
     *
     */
    UFUNCTION()
    virtual bool SetupConstraintsAndPhysics();

    /**
     * @brief Setup #UDifferentialDriveComponent
     *
     */
    UFUNCTION()
    virtual void SetupWheelDrives();

    //! pass to #UDifferentialDriveComponent.
    //! @todo get from static meshes.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelRadius = 3.3f;

    //! pass to #UDifferentialDriveComponent.
    //! @todo get data from links.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelSeparationHalf = 7.9f;
};
