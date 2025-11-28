/**
 * @file RRTwoPointInterpolation.h
 * @brief UE wrapper for TwoPointInterpolation trajectory planning
 * @copyright Copyright 2025 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Components/ActorComponent.h"
#include "Kismet/BlueprintFunctionLibrary.h"
// #include "CoreMinimal.h"
// #include "UObject/NoExportTypes.h"

// ThirdParty
#include "two_points_interpolation_constant_acc.hpp"

#include "RRTwoPointInterpolation.generated.h"

/**
 * @brief Trajectory point data structure
 */
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FTrajectoryPoint
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory")
    float Position = 0.0f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory")
    float Velocity = 0.0f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory")
    float Acceleration = 0.0f;

    FTrajectoryPoint() = default;
    
    FTrajectoryPoint(float InPosition, float InVelocity, float InAcceleration)
        : Position(InPosition), Velocity(InVelocity), Acceleration(InAcceleration) {}
};

/**
 * @brief UE wrapper for TwoPointInterpolation for position trajectories
 */
UCLASS(BlueprintType, Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API URRTwoPointInterpolation : public UObject
{
    GENERATED_BODY()

public:
    URRTwoPointInterpolation();

    /**
     * @brief Set starting parameters
     * @param StartPosition Initial position
     * @param StartVelocity Initial velocity (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void SetStartPoint(float StartPosition, float StartVelocity = 0.0f);
    
    /**
     * @brief Set target parameters
     * @param EndPosition Target position
     * @param EndVelocity Target velocity (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void SetTargetPoint(float EndPosition, float EndVelocity = 0.0f);

    /**
     * @brief Set trajectory constraints
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param MaxDeceleration Maximum deceleration (optional, defaults to MaxAcceleration)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void SetConstraints(float MaxAcceleration, float MaxVelocity, float MaxDeceleration = -1.0f);

    /**
     * @brief Initialize trajectory with all parameters
     * @param StartPosition Initial position
     * @param EndPosition Target position
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param StartTime Initial time (default: 0)
     * @param StartVelocity Initial velocity (default: 0)
     * @param EndVelocity Target velocity (default: 0)
     * @param MaxDeceleration Maximum deceleration (optional)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void Initialize(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                   float StartTime = 0.0f, float StartVelocity = 0.0f, float EndVelocity = 0.0f, float MaxDeceleration = -1.0f);

    /**
     * @brief Calculate trajectory with current parameters
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    float CalculateTrajectory();

    /**
     * @brief Calculate trajectory with all parameters in one call
     * @param StartPosition Initial position
     * @param EndPosition Target position
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param StartTime Initial time (default: 0)
     * @param StartVelocity Initial velocity (default: 0)
     * @param EndVelocity Target velocity (default: 0)
     * @param MaxDeceleration Maximum deceleration (optional)
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    float CalculateTrajectoryWithParams(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                                       float StartTime = 0.0f, float StartVelocity = 0.0f, float EndVelocity = 0.0f, float MaxDeceleration = -1.0f, bool bVerbose = false);

    /**
     * @brief Get trajectory point at specified time
     * @param Time Time to sample
     * @return Trajectory point (position, velocity, acceleration)
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation")
    FTrajectoryPoint GetPointAtTime(float Time) const;

    /**
     * @brief Check if trajectory is valid and initialized
     * @return True if trajectory is ready for sampling
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation")
    bool IsInitialized() const;

protected:
    mutable TwoPointInterpolation Interpolator;
    
    // Cache for avoiding unnecessary recalculation (only target-related parameters)
    float CachedEndPosition = 0.0f;
    float CachedEndVelocity = 0.0f;
    float CachedMaxAcceleration = 0.0f;
    float CachedMaxVelocity = 0.0f;
    float CachedMaxDeceleration = 0.0f;
    float CachedDuration = -1.0f;
    bool bCacheValid = false;
};

/**
 * @brief UE wrapper for TwoAngleInterpolation for angular trajectories
 */
UCLASS(BlueprintType, Blueprintable)
class RAPYUTASIMULATIONPLUGINS_API URRTwoAngleInterpolation : public UObject
{
    GENERATED_BODY()

public:
    URRTwoAngleInterpolation();

    /**
     * @brief Set starting parameters
     * @param StartAngle Initial angle in degrees
     * @param StartAngularVelocity Initial angular velocity in degrees/sec (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void SetStartPoint(float StartAngle, float StartAngularVelocity = 0.0f);
    
    /**
     * @brief Set target parameters
     * @param EndAngle Target angle in degrees
     * @param EndAngularVelocity Target angular velocity in degrees/sec (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void SetTargetPoint(float EndAngle, float EndAngularVelocity = 0.0f);

    /**
     * @brief Set trajectory constraints
     * @param MaxAngularAcceleration Maximum angular acceleration in degrees/sec²
     * @param MaxAngularVelocity Maximum angular velocity in degrees/sec
     * @param MaxAngularDeceleration Maximum angular deceleration (optional, defaults to MaxAngularAcceleration)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void SetConstraints(float MaxAngularAcceleration, float MaxAngularVelocity, float MaxAngularDeceleration = -1.0f);

    /**
     * @brief Initialize trajectory with all parameters
     * @param StartAngle Initial angle in degrees
     * @param EndAngle Target angle in degrees
     * @param MaxAngularAcceleration Maximum angular acceleration in degrees/sec²
     * @param MaxAngularVelocity Maximum angular velocity in degrees/sec
     * @param StartTime Initial time (default: 0)
     * @param StartAngularVelocity Initial angular velocity in degrees/sec (default: 0)
     * @param EndAngularVelocity Target angular velocity in degrees/sec (default: 0)
     * @param MaxAngularDeceleration Maximum angular deceleration in degrees/sec² (optional)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void Initialize(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                   float StartTime = 0.0f, float StartAngularVelocity = 0.0f, float EndAngularVelocity = 0.0f, float MaxAngularDeceleration = -1.0f);

    /**
     * @brief Calculate trajectory with current parameters
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    float CalculateTrajectory();

    /**
     * @brief Calculate trajectory with all parameters in one call
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
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    float CalculateTrajectoryWithParams(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                                       float StartTime = 0.0f, float StartAngularVelocity = 0.0f, float EndAngularVelocity = 0.0f, float MaxAngularDeceleration = -1.0f);

    /**
     * @brief Get trajectory point at specified time
     * @param Time Time to sample
     * @param bNormalizeOutput Whether to normalize output angle to [-180, 180] range
     * @return Trajectory point (position in degrees, velocity in degrees/sec, acceleration in degrees/sec²)
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Angle Interpolation")
    FTrajectoryPoint GetPointAtTime(float Time, bool bNormalizeOutput = true) const;

    /**
     * @brief Check if trajectory is valid and initialized
     * @return True if trajectory is ready for sampling
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Angle Interpolation")
    bool IsInitialized() const;

protected:
    mutable TwoAngleInterpolation Interpolator;
};

/**
 * @brief Component wrapper for TwoPointInterpolation trajectory planning
 * This component allows easy use of trajectory interpolation in Blueprints
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRTwoPointInterpolationComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    URRTwoPointInterpolationComponent();

    /**
     * @brief Set starting point parameters
     * @param StartPosition Initial position
     * @param StartVelocity Initial velocity (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void SetStartPoint(float StartPosition, float StartVelocity = 0.0f);

    /**
     * @brief Set target parameters
     * @param EndPosition Target position
     * @param EndVelocity Target velocity (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void SetTargetPoint(float EndPosition, float EndVelocity = 0.0f);

    /**
     * @brief Set trajectory constraints
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param MaxDeceleration Maximum deceleration (optional, defaults to MaxAcceleration)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void SetConstraints(float MaxAcceleration, float MaxVelocity, float MaxDeceleration = -1.0f);

    /**
     * @brief Initialize trajectory with all parameters
     * @param StartPosition Initial position
     * @param EndPosition Target position
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param StartTime Initial time (default: 0)
     * @param StartVelocity Initial velocity (default: 0)
     * @param EndVelocity Target velocity (default: 0)
     * @param MaxDeceleration Maximum deceleration (optional)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    void Initialize(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                   float StartTime = 0.0f, float StartVelocity = 0.0f, float EndVelocity = 0.0f, float MaxDeceleration = -1.0f);

    /**
     * @brief Calculate trajectory with current parameters
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    float CalculateTrajectory();

    /**
     * @brief Calculate trajectory with all parameters in one call
     * @param StartPosition Initial position
     * @param EndPosition Target position
     * @param MaxAcceleration Maximum acceleration
     * @param MaxVelocity Maximum velocity
     * @param StartTime Initial time (default: 0)
     * @param StartVelocity Initial velocity (default: 0)
     * @param EndVelocity Target velocity (default: 0)
     * @param MaxDeceleration Maximum deceleration (optional)
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Point Interpolation")
    float CalculateTrajectoryWithParams(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                                       float StartTime = 0.0f, float StartVelocity = 0.0f, float EndVelocity = 0.0f, float MaxDeceleration = -1.0f);

    /**
     * @brief Get trajectory point at specified time
     * @param Time Time to sample
     * @return Trajectory point (position, velocity, acceleration)
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation")
    FTrajectoryPoint GetPointAtTime(float Time) const;

    /**
     * @brief Check if trajectory is valid and initialized
     * @return True if trajectory is ready for sampling
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation")
    bool IsInitialized() const;

    /**
     * @brief Get the underlying interpolation object (for advanced use)
     * @return Pointer to the interpolation object
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Point Interpolation")
    URRTwoPointInterpolation* GetInterpolationObject() const { return PositionInterpolation; }

protected:
    //! Position interpolation object
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Two Point Interpolation")
    URRTwoPointInterpolation* PositionInterpolation = nullptr;
};

/**
 * @brief Component wrapper for TwoAngleInterpolation trajectory planning
 * This component allows easy use of angular trajectory interpolation in Blueprints
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRTwoAngleInterpolationComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    URRTwoAngleInterpolationComponent();

    /**
     * @brief Set starting point parameters
     * @param StartAngle Initial angle (degrees)
     * @param StartAngularVelocity Initial angular velocity (degrees/s) (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void SetStartPoint(float StartAngle, float StartAngularVelocity = 0.0f);

    /**
     * @brief Set target parameters
     * @param EndAngle Target angle (degrees)
     * @param EndAngularVelocity Target angular velocity (degrees/s) (default: 0)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void SetTargetPoint(float EndAngle, float EndAngularVelocity = 0.0f);

    /**
     * @brief Set trajectory constraints
     * @param MaxAngularAcceleration Maximum angular acceleration (degrees/s²)
     * @param MaxAngularVelocity Maximum angular velocity (degrees/s)
     * @param MaxAngularDeceleration Maximum angular deceleration (degrees/s²) (optional)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void SetConstraints(float MaxAngularAcceleration, float MaxAngularVelocity, float MaxAngularDeceleration = -1.0f);

    /**
     * @brief Initialize trajectory with all parameters
     * @param StartAngle Initial angle (degrees)
     * @param EndAngle Target angle (degrees)
     * @param MaxAngularAcceleration Maximum angular acceleration (degrees/s²)
     * @param MaxAngularVelocity Maximum angular velocity (degrees/s)
     * @param StartTime Initial time (default: 0)
     * @param StartAngularVelocity Initial angular velocity (degrees/s) (default: 0)
     * @param EndAngularVelocity Target angular velocity (degrees/s) (default: 0)
     * @param MaxAngularDeceleration Maximum angular deceleration (degrees/s²) (optional)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    void Initialize(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                   float StartTime = 0.0f, float StartAngularVelocity = 0.0f, float EndAngularVelocity = 0.0f, float MaxAngularDeceleration = -1.0f);

    /**
     * @brief Calculate trajectory with current parameters
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    float CalculateTrajectory();

    /**
     * @brief Calculate trajectory with all parameters in one call
     * @param StartAngle Initial angle (degrees)
     * @param EndAngle Target angle (degrees)
     * @param MaxAngularAcceleration Maximum angular acceleration (degrees/s²)
     * @param MaxAngularVelocity Maximum angular velocity (degrees/s)
     * @param StartTime Initial time (default: 0)
     * @param StartAngularVelocity Initial angular velocity (degrees/s) (default: 0)
     * @param EndAngularVelocity Target angular velocity (degrees/s) (default: 0)
     * @param MaxAngularDeceleration Maximum angular deceleration (degrees/s²) (optional)
     * @return Total duration of the trajectory (negative if error)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Angle Interpolation")
    float CalculateTrajectoryWithParams(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                                       float StartTime = 0.0f, float StartAngularVelocity = 0.0f, float EndAngularVelocity = 0.0f, float MaxAngularDeceleration = -1.0f);

    /**
     * @brief Get trajectory point at specified time
     * @param Time Time to sample
     * @return Trajectory point (angle, angular velocity, angular acceleration in degrees)
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Angle Interpolation")
    FTrajectoryPoint GetPointAtTime(float Time) const;

    /**
     * @brief Check if trajectory is valid and initialized
     * @return True if trajectory is ready for sampling
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Angle Interpolation")
    bool IsInitialized() const;

    /**
     * @brief Get the underlying interpolation object (for advanced use)
     * @return Pointer to the interpolation object
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Two Angle Interpolation")
    URRTwoAngleInterpolation* GetInterpolationObject() const { return AngleInterpolation; }

protected:
    //! Angle interpolation object
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Two Angle Interpolation")
    URRTwoAngleInterpolation* AngleInterpolation = nullptr;
};

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