/**
 * @file RRBaseLidarComponent.h
 * @brief Base ROS2 LIDAR Component class. Other lidar class should inherit from this class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// std
#include <random>

// UE
#include "Components/StaticMeshComponent.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "RRROS2BaseSensorComponent.h"

#include "RRBaseLidarComponent.generated.h"

#define TRACE_ASYNC 1

class URRROS2LidarPublisher;

/**
 * @brief Base ROS2 LIDAR Component class. Other lidar class should inherit from this class.
 * 
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRBaseLidarComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    URRBaseLidarComponent();

protected:
    virtual void BeginPlay() override;

public:
    /**
     * @brief Return true if laser hits the target actor. This method should be overwritten by child class.
     * @param TargetActor 
     * @return true 
     * @return false 
     */
    UFUNCTION(BlueprintCallable)
    virtual bool Visible(AActor* TargetActor)
    {
        checkNoEntry();
        return false;
    }

    /**
     * @brief Get #RecordedHits and #TimeOfLastScan.
     * adding the rest of the necessary information might be tedious
     * eventually split into multiple getters
     *
     * @param OutHits 
     * @param OutTime 
     */
    UFUNCTION(BlueprintCallable)
    void GetData(TArray<FHitResult>& OutHits, float& OutTime) const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NSamplesPerScan = 360;

    //! [degrees] scan goes from StartAngle to StartAngle+FOVHorizontal
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartAngle = 0.f;

    //! [degrees] scan goes from StartAngle to StartAngle+FOVHorizontal
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float FOVHorizontal = 360.f;

    //! [m]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MinRange = 12.f;

    //! [m]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxRange = 350.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FLinearColor ColorMiss = FColor(255, 127, 0, 255);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FLinearColor ColorMin = FColor(255, 0, 0, 255);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FLinearColor ColorMid = FColor(255, 0, 0, 255);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FColor ColorMax = FColor(255, 255, 255, 255);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float TimeOfLastScan = 0.f;

    // [degrees]
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float DHAngle = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float PositionalNoiseMean = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float PositionalNoiseVariance = 1.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float IntensityNoiseMean = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float IntensityNoiseVariance = .1f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    uint8 BWithNoise : 1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TArray<FHitResult> RecordedHits;

#if TRACE_ASYNC
    TArray<FTraceHandle> TraceHandles;
#endif

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bShowLidarRays = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool ShowLidarRayMisses = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityNonReflective = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityReflective = 6000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityMin = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityMax = 10000.f;

    FLinearColor InterpColorFromIntensity(const float InIntensity);

protected:
    UPROPERTY()
    float Dt = 0.f;

    //! C++11 RNG for noise
    std::random_device Rng;

    //! C++11 RNG for noise
    std::mt19937 Gen = std::mt19937{Rng()};

    std::normal_distribution<> GaussianRNGPosition;

    std::normal_distribution<> GaussianRNGIntensity;

    FLinearColor InterpolateColor(float InX);
    static float GetIntensityFromDist(float InBaseIntensity, float InDistance);
};
