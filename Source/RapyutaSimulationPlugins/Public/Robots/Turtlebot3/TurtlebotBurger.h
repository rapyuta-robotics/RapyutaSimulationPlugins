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
#include "Sensors/RR2DLidarComponent.h"

#include "TurtlebotBurger.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogTurtlebotBurger, Log, All);

/**
 * @brief Example of child class of #ARRBaseRobot
 * Uses #UDifferentialDriveComponent and has #URR2DLidarComponent.
 * This class is designed to be inheritted from Blueprint class to be assigned UStaticMeshComponent.
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ATurtlebotBurger : public ARRBaseRobot
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

    /**
     * @brief calls #SetupWheelDrives
     * 
     */
    virtual void PostInitializeComponents() override;

    /**
     * @brief Create UStaticMeshComponent, create UPhysicsConstraintComponent, and calls #SetupConstraintsAndPhysics to setup physics constraints.
     * 
     */
    UFUNCTION()
    void SetupBody();

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
    UPhysicsConstraintComponent* Base_WheelLeft = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPhysicsConstraintComponent* Base_WheelRight = nullptr;

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
     * @brief Setup material, relative location, anugular/linear limits, drive params.
     * 
     */
    UFUNCTION()
    void SetupConstraintsAndPhysics();

    /**
     * @brief Setup #UDifferentialDriveComponent
     * 
     */
    UFUNCTION()
    void SetupWheelDrives();

    //! pass to #UDifferentialDriveComponent. 
    //! @todo get from static meshes.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelRadius = 3.3f;

    //! pass to #UDifferentialDriveComponent. 
    //! @todo get data from links.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WheelSeparationHalf = 7.9f;
};
