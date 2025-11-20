/**
 * @file RRTwoPointInterpolationLibrary.h
 * @brief Blueprint function library for TwoPointInterpolation utilities
 * @copyright Copyright 2025 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

// Local
#include "Tools/RRTwoPointInterpolation.h"

#include "RRTwoPointInterpolationLibrary.generated.h"

/**
 * @brief Blueprint function library for trajectory interpolation utilities
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRTwoPointInterpolationLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /**
     * @brief Quick position trajectory calculation with all parameters
     * @param StartPosition Initial position
     * @param EndPosition Target position  
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param StartTime Initial time (default: 0)
     * @param StartVelocity Initial velocity (default: 0)
     * @param EndVelocity Target velocity (default: 0)
     * @param MaxDeceleration Maximum deceleration (optional, defaults to MaxAcceleration)
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation Library")
    static float CalculatePositionTrajectoryDuration(float StartPosition, float EndPosition, 
                                                    float MaxAcceleration, float MaxVelocity,
                                                    float StartTime = 0.0f, float StartVelocity = 0.0f, 
                                                    float EndVelocity = 0.0f, float MaxDeceleration = -1.0f);

    /**
     * @brief Quick angle trajectory calculation with all parameters  
     * @param StartAngle Initial angle in degrees
     * @param EndAngle Target angle in degrees
     * @param MaxAngularAcceleration Maximum angular acceleration in degrees/sec²
     * @param MaxAngularVelocity Maximum angular velocity in degrees/sec
     * @param StartTime Initial time (default: 0)
     * @param StartAngularVelocity Initial angular velocity in degrees/sec (default: 0)
     * @param EndAngularVelocity Target angular velocity in degrees/sec (default: 0)
     * @param MaxAngularDeceleration Maximum angular deceleration in degrees/sec² (optional)
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation Library")
    static float CalculateAngleTrajectoryDuration(float StartAngle, float EndAngle,
                                                 float MaxAngularAcceleration, float MaxAngularVelocity,
                                                 float StartTime = 0.0f, float StartAngularVelocity = 0.0f,
                                                 float EndAngularVelocity = 0.0f, float MaxAngularDeceleration = -1.0f);

    /**
     * @brief Get position trajectory point at specific time
     * @param StartPosition Initial position
     * @param EndPosition Target position
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param Time Time to sample
     * @param StartTime Initial time (default: 0)
     * @param StartVelocity Initial velocity (default: 0)
     * @param EndVelocity Target velocity (default: 0)
     * @param MaxDeceleration Maximum deceleration (optional)
     * @return Trajectory point at specified time
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation Library")
    static FTrajectoryPoint GetPositionTrajectoryPointAtTime(float StartPosition, float EndPosition,
                                                           float MaxAcceleration, float MaxVelocity, float Time,
                                                           float StartTime = 0.0f, float StartVelocity = 0.0f,
                                                           float EndVelocity = 0.0f, float MaxDeceleration = -1.0f);

    /**
     * @brief Get angle trajectory point at specific time
     * @param StartAngle Initial angle in degrees
     * @param EndAngle Target angle in degrees
     * @param MaxAngularAcceleration Maximum angular acceleration in degrees/sec²
     * @param MaxAngularVelocity Maximum angular velocity in degrees/sec
     * @param Time Time to sample
     * @param StartTime Initial time (default: 0)
     * @param StartAngularVelocity Initial angular velocity in degrees/sec (default: 0)
     * @param EndAngularVelocity Target angular velocity in degrees/sec (default: 0)
     * @param MaxAngularDeceleration Maximum angular deceleration in degrees/sec² (optional)
     * @param bNormalizeOutput Whether to normalize output angle (default: true)
     * @return Trajectory point at specified time (in degrees)
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation Library")
    static FTrajectoryPoint GetAngleTrajectoryPointAtTime(float StartAngle, float EndAngle,
                                                         float MaxAngularAcceleration, float MaxAngularVelocity, float Time,
                                                         float StartTime = 0.0f, float StartAngularVelocity = 0.0f,
                                                         float EndAngularVelocity = 0.0f, float MaxAngularDeceleration = -1.0f,
                                                         bool bNormalizeOutput = true);

    /**
     * @brief Validate trajectory parameters
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param MaxDeceleration Maximum deceleration (optional)
     * @return True if parameters are valid
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation Library")
    static bool ValidateTrajectoryParameters(float MaxAcceleration, float MaxVelocity, float MaxDeceleration = -1.0f);
};