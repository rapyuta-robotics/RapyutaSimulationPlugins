/**
 * @file RRTwoPointInterpolation.cpp
 * @brief UE wrapper for TwoPointInterpolation trajectory planning implementation
 * @copyright Copyright 2025 Rapyuta Robotics Co., Ltd.
 */

#include "Tools/RRTwoPointInterpolation.h"

// UE
#include "Engine/Engine.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"

URRTwoPointInterpolation::URRTwoPointInterpolation()
    : Interpolator(false) // Initialize with debug disabled
{
}

void URRTwoPointInterpolation::SetStartPoint(float StartPosition, float StartVelocity)
{
    try 
    {
        // Convert UE units (cm, cm/s) to library units (m, m/s)
        double StartPositionM = static_cast<double>(StartPosition) / 100.0;
        double StartVelocityM = static_cast<double>(StartVelocity) / 100.0;
        
        Interpolator.setInitial(0.0, StartPositionM, StartVelocityM);
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Start point set: Position=%.3f cm, Velocity=%.3f cm/s"), StartPosition, StartVelocity);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("SetStartPoint failed: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void URRTwoPointInterpolation::SetTargetPoint(float EndPosition, float EndVelocity)
{
    try 
    {
        // Convert UE units (cm, cm/s) to library units (m, m/s)
        double EndPositionM = static_cast<double>(EndPosition) / 100.0;
        double EndVelocityM = static_cast<double>(EndVelocity) / 100.0;
        
        Interpolator.setPoint(EndPositionM, EndVelocityM);
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Target point set: Position=%.3f cm, Velocity=%.3f cm/s"), EndPosition, EndVelocity);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("SetTargetPoint failed: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void URRTwoPointInterpolation::SetConstraints(float MaxAcceleration, float MaxVelocity, float MaxDeceleration)
{
    try 
    {
        // Convert UE units (cm/s², cm/s) to library units (m/s², m/s)
        double MaxAccelerationM = static_cast<double>(MaxAcceleration) / 100.0;
        double MaxVelocityM = static_cast<double>(MaxVelocity) / 100.0;
        double MaxDecelerationM = static_cast<double>(MaxDeceleration) / 100.0;
        
        Interpolator.setConstraints(MaxAccelerationM, MaxVelocityM, MaxDecelerationM);
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Constraints set: MaxAccel=%.3f cm/s², MaxVel=%.3f cm/s, MaxDecel=%.3f cm/s²"), 
                         MaxAcceleration, MaxVelocity, MaxDeceleration);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("SetConstraints failed: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void URRTwoPointInterpolation::Initialize(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                                         float StartTime, float StartVelocity, float EndVelocity, float MaxDeceleration)
{
    try 
    {
        // Convert UE units to library units
        double StartPositionM = static_cast<double>(StartPosition) / 100.0;
        double EndPositionM = static_cast<double>(EndPosition) / 100.0;
        double MaxAccelerationM = static_cast<double>(MaxAcceleration) / 100.0;
        double MaxVelocityM = static_cast<double>(MaxVelocity) / 100.0;
        double StartVelocityM = static_cast<double>(StartVelocity) / 100.0;
        double EndVelocityM = static_cast<double>(EndVelocity) / 100.0;
        double MaxDecelerationM = static_cast<double>(MaxDeceleration) / 100.0;
        
        Interpolator.init(StartPositionM, EndPositionM, MaxAccelerationM, MaxVelocityM,
                         static_cast<double>(StartTime), StartVelocityM, EndVelocityM, MaxDecelerationM);
        
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("TwoPointInterpolation initialized successfully"));
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("TwoPointInterpolation initialization failed: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

float URRTwoPointInterpolation::CalculateTrajectory()
{
    try 
    {
        double duration = Interpolator.calcTrajectory();
        float result = static_cast<float>(duration);
        
        if (result < 0.0f)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Trajectory calculation failed with duration: %f"), result);
        }
        
        return result;
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("CalculateTrajectory failed: %s"), UTF8_TO_TCHAR(e.what()));
        return -1.0f;
    }
}

float URRTwoPointInterpolation::CalculateTrajectoryWithParams(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                                                             float StartTime, float StartVelocity, float EndVelocity, float MaxDeceleration, bool bVerbose)
{
    // Check if parameters are the same as cached ones (only target-related parameters matter)
    if (bCacheValid && 
        FMath::IsNearlyEqual(CachedEndPosition, EndPosition, 0.01f) &&
        FMath::IsNearlyEqual(CachedEndVelocity, EndVelocity, 0.01f) &&
        FMath::IsNearlyEqual(CachedMaxAcceleration, MaxAcceleration, 0.01f) &&
        FMath::IsNearlyEqual(CachedMaxVelocity, MaxVelocity, 0.01f) &&
        FMath::IsNearlyEqual(CachedMaxDeceleration, MaxDeceleration, 0.01f))
    {
        if (bVerbose)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("TrajectoryCalc CACHE: Using cached result Duration=%.3fs (Same target parameters: EndPos=%.2fcm EndVel=%.2fcm/s)"), CachedDuration, EndPosition, EndVelocity);
        }
        return CachedDuration;
    }

    try 
    {
        // Convert UE units to library units
        double StartPositionM = static_cast<double>(StartPosition) / 100.0;
        double EndPositionM = static_cast<double>(EndPosition) / 100.0;
        double MaxAccelerationM = static_cast<double>(MaxAcceleration) / 100.0;
        double MaxVelocityM = static_cast<double>(MaxVelocity) / 100.0;
        double StartVelocityM = static_cast<double>(StartVelocity) / 100.0;
        double EndVelocityM = static_cast<double>(EndVelocity) / 100.0;
        double MaxDecelerationM = static_cast<double>(MaxDeceleration) / 100.0;
        
        Interpolator.init(StartPositionM, EndPositionM, MaxAccelerationM, MaxVelocityM,
                         static_cast<double>(StartTime), StartVelocityM, EndVelocityM, MaxDecelerationM);
        
        double duration = Interpolator.calcTrajectory();
        float result = static_cast<float>(duration);
        
        if (bVerbose)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Log, 
                TEXT("TrajectoryCalc: Start=%.2fcm End=%.2fcm Dist=%.2fcm | MaxAccel=%.2fcm/s² MaxVel=%.2fcm/s MaxDecel=%.2fcm/s² | StartTime=%.3fs StartVel=%.2fcm/s EndVel=%.2fcm/s | Duration=%.3fs"),
                StartPosition, EndPosition, FMath::Abs(EndPosition - StartPosition), 
                MaxAcceleration, MaxVelocity, MaxDeceleration, 
                StartTime, StartVelocity, EndVelocity, 
                result);
        }
        
        // Update cache (only target-related parameters)
        CachedEndPosition = EndPosition;
        CachedEndVelocity = EndVelocity;
        CachedMaxAcceleration = MaxAcceleration;
        CachedMaxVelocity = MaxVelocity;
        CachedMaxDeceleration = MaxDeceleration;
        CachedDuration = result;
        bCacheValid = true;
        
        return result;
    }
    catch (const std::exception& e)
    {
        bCacheValid = false;  // Invalidate cache on error
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("TrajectoryCalc ERROR: Start=%.2fcm End=%.2fcm Dist=%.2fcm | MaxAccel=%.2fcm/s² MaxVel=%.2fcm/s MaxDecel=%.2fcm/s² | StartTime=%.3fs StartVel=%.2fcm/s EndVel=%.2fcm/s | Error: %s"), 
                         StartPosition, EndPosition, FMath::Abs(EndPosition - StartPosition), 
                         MaxAcceleration, MaxVelocity, MaxDeceleration, 
                         StartTime, StartVelocity, EndVelocity, 
                         UTF8_TO_TCHAR(e.what()));
        return -1.0f;
    }
}

FTrajectoryPoint URRTwoPointInterpolation::GetPointAtTime(float Time) const
{
    try 
    {
        std::vector<double> result = Interpolator.getPoint(static_cast<double>(Time));
        
        if (result.size() >= 3)
        {
            // Convert library units (m, m/s, m/s²) back to UE units (cm, cm/s, cm/s²)
            float PositionCm = static_cast<float>(result[0] * 100.0);
            float VelocityCm = static_cast<float>(result[1] * 100.0);
            float AccelerationCm = static_cast<float>(result[2] * 100.0);
            
            return FTrajectoryPoint(PositionCm, VelocityCm, AccelerationCm);
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("getPoint returned insufficient data"));
            return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
        }
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("GetPointAtTime failed: %s"), UTF8_TO_TCHAR(e.what()));
        return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
    }
}

bool URRTwoPointInterpolation::IsInitialized() const
{
    // Now we can use the library's own method thanks to mutable
    return Interpolator.isInitialized();
}

// URRTwoAngleInterpolation Implementation

URRTwoAngleInterpolation::URRTwoAngleInterpolation()
    : Interpolator(false) // Initialize with debug disabled
{
}

void URRTwoAngleInterpolation::SetStartPoint(float StartAngle, float StartAngularVelocity)
{
    try 
    {
        Interpolator.setInitial(0.0, FMath::DegreesToRadians(static_cast<double>(StartAngle)), 
                               FMath::DegreesToRadians(static_cast<double>(StartAngularVelocity)));
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Angle start point set: Angle=%.3f deg, AngularVel=%.3f deg/s"), StartAngle, StartAngularVelocity);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("SetStartPoint failed for angle: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void URRTwoAngleInterpolation::SetTargetPoint(float EndAngle, float EndAngularVelocity)
{
    try 
    {
        Interpolator.setPoint(FMath::DegreesToRadians(static_cast<double>(EndAngle)), 
                             FMath::DegreesToRadians(static_cast<double>(EndAngularVelocity)));
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Angle target point set: Angle=%.3f deg, AngularVel=%.3f deg/s"), EndAngle, EndAngularVelocity);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("SetTargetPoint failed for angle: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void URRTwoAngleInterpolation::SetConstraints(float MaxAngularAcceleration, float MaxAngularVelocity, float MaxAngularDeceleration)
{
    try 
    {
        const double RealMaxDeceleration = (MaxAngularDeceleration >= 0) ? MaxAngularDeceleration : MaxAngularAcceleration;
        
        Interpolator.setConstraints(FMath::DegreesToRadians(static_cast<double>(MaxAngularAcceleration)),
                                    FMath::DegreesToRadians(static_cast<double>(MaxAngularVelocity)),
                                    FMath::DegreesToRadians(static_cast<double>(RealMaxDeceleration)));
        
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Angle constraints set: MaxAngularAccel=%.3f deg/s², MaxAngularVel=%.3f deg/s, MaxAngularDecel=%.3f deg/s²"), 
                         MaxAngularAcceleration, MaxAngularVelocity, RealMaxDeceleration);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("SetConstraints failed for angle: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

void URRTwoAngleInterpolation::Initialize(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                                         float StartTime, float StartAngularVelocity, float EndAngularVelocity, float MaxAngularDeceleration)
{
    try 
    {
        // Convert degrees to radians for the library
        double StartAngleRad = FMath::DegreesToRadians(static_cast<double>(StartAngle));
        double EndAngleRad = FMath::DegreesToRadians(static_cast<double>(EndAngle));
        double MaxAccelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularAcceleration));
        double MaxVelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularVelocity));
        double StartVelRad = FMath::DegreesToRadians(static_cast<double>(StartAngularVelocity));
        double EndVelRad = FMath::DegreesToRadians(static_cast<double>(EndAngularVelocity));
        double DecelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularDeceleration));
        
        Interpolator.init(StartAngleRad, EndAngleRad, MaxAccelRad, MaxVelRad,
                         static_cast<double>(StartTime), StartVelRad, EndVelRad, DecelRad);
        
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("TwoAngleInterpolation initialized successfully"));
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("TwoAngleInterpolation initialization failed: %s"), UTF8_TO_TCHAR(e.what()));
    }
}

float URRTwoAngleInterpolation::CalculateTrajectory()
{
    try 
    {
        double duration = Interpolator.TwoPointInterpolation::calcTrajectory();
        float result = static_cast<float>(duration);
        
        if (result < 0.0f)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Angle trajectory calculation failed with duration: %f"), result);
        }
        
        return result;
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("CalculateTrajectory failed: %s"), UTF8_TO_TCHAR(e.what()));
        return -1.0f;
    }
}

float URRTwoAngleInterpolation::CalculateTrajectoryWithParams(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                                                             float StartTime, float StartAngularVelocity, float EndAngularVelocity, float MaxAngularDeceleration)
{
    try 
    {
        // Convert degrees to radians for the library
        double StartAngleRad = FMath::DegreesToRadians(static_cast<double>(StartAngle));
        double EndAngleRad = FMath::DegreesToRadians(static_cast<double>(EndAngle));
        double MaxAccelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularAcceleration));
        double MaxVelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularVelocity));
        double StartVelRad = FMath::DegreesToRadians(static_cast<double>(StartAngularVelocity));
        double EndVelRad = FMath::DegreesToRadians(static_cast<double>(EndAngularVelocity));
        double DecelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularDeceleration));
        
        Interpolator.init(StartAngleRad, EndAngleRad, MaxAccelRad, MaxVelRad,
                         static_cast<double>(StartTime), StartVelRad, EndVelRad, DecelRad);
        
        double duration = Interpolator.TwoPointInterpolation::calcTrajectory();
        return static_cast<float>(duration);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("CalculateTrajectoryWithParams failed for angle: %s"), UTF8_TO_TCHAR(e.what()));
        return -1.0f;
    }
}

FTrajectoryPoint URRTwoAngleInterpolation::GetPointAtTime(float Time, bool bNormalizeOutput) const
{
    try 
    {
        std::vector<double> result = Interpolator.getPoint(static_cast<double>(Time), bNormalizeOutput);
        
        if (result.size() >= 3)
        {
            // Convert radians back to degrees
            float PositionDeg = FMath::RadiansToDegrees(static_cast<float>(result[0]));
            float VelocityDeg = FMath::RadiansToDegrees(static_cast<float>(result[1]));
            float AccelerationDeg = FMath::RadiansToDegrees(static_cast<float>(result[2]));
            
            return FTrajectoryPoint(PositionDeg, VelocityDeg, AccelerationDeg);
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("getPoint returned insufficient data for angle"));
            return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
        }
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("GetPointAtTime failed for angle: %s"), UTF8_TO_TCHAR(e.what()));
        return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
    }
}

bool URRTwoAngleInterpolation::IsInitialized() const
{
    // Now we can use the library's own method thanks to mutable
    return Interpolator.isInitialized();
}

// URRTwoPointInterpolationComponent Implementation

URRTwoPointInterpolationComponent::URRTwoPointInterpolationComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    
    // Create the interpolation object
    PositionInterpolation = CreateDefaultSubobject<URRTwoPointInterpolation>(TEXT("PositionInterpolation"));
}

void URRTwoPointInterpolationComponent::SetStartPoint(float StartPosition, float StartVelocity)
{
    if (PositionInterpolation)
    {
        PositionInterpolation->SetStartPoint(StartPosition, StartVelocity);
    }
}

void URRTwoPointInterpolationComponent::SetTargetPoint(float EndPosition, float EndVelocity)
{
    if (PositionInterpolation)
    {
        PositionInterpolation->SetTargetPoint(EndPosition, EndVelocity);
    }
}

void URRTwoPointInterpolationComponent::SetConstraints(float MaxAcceleration, float MaxVelocity, float MaxDeceleration)
{
    if (PositionInterpolation)
    {
        PositionInterpolation->SetConstraints(MaxAcceleration, MaxVelocity, MaxDeceleration);
    }
}

void URRTwoPointInterpolationComponent::Initialize(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                                                  float StartTime, float StartVelocity, float EndVelocity, float MaxDeceleration)
{
    if (PositionInterpolation)
    {
        PositionInterpolation->Initialize(StartPosition, EndPosition, MaxAcceleration, MaxVelocity,
                                         StartTime, StartVelocity, EndVelocity, MaxDeceleration);
    }
}

float URRTwoPointInterpolationComponent::CalculateTrajectory()
{
    if (PositionInterpolation)
    {
        return PositionInterpolation->CalculateTrajectory();
    }
    return -1.0f;
}

float URRTwoPointInterpolationComponent::CalculateTrajectoryWithParams(float StartPosition, float EndPosition, float MaxAcceleration, float MaxVelocity,
                                                                       float StartTime, float StartVelocity, float EndVelocity, float MaxDeceleration)
{
    if (PositionInterpolation)
    {
        return PositionInterpolation->CalculateTrajectoryWithParams(StartPosition, EndPosition, MaxAcceleration, MaxVelocity,
                                                                    StartTime, StartVelocity, EndVelocity, MaxDeceleration);
    }
    return -1.0f;
}

FTrajectoryPoint URRTwoPointInterpolationComponent::GetPointAtTime(float Time) const
{
    if (PositionInterpolation)
    {
        return PositionInterpolation->GetPointAtTime(Time);
    }
    return FTrajectoryPoint();
}

bool URRTwoPointInterpolationComponent::IsInitialized() const
{
    if (PositionInterpolation)
    {
        return PositionInterpolation->IsInitialized();
    }
    return false;
}

// URRTwoAngleInterpolationComponent Implementation

URRTwoAngleInterpolationComponent::URRTwoAngleInterpolationComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    
    // Create the interpolation object
    AngleInterpolation = CreateDefaultSubobject<URRTwoAngleInterpolation>(TEXT("AngleInterpolation"));
}

void URRTwoAngleInterpolationComponent::SetStartPoint(float StartAngle, float StartAngularVelocity)
{
    if (AngleInterpolation)
    {
        AngleInterpolation->SetStartPoint(StartAngle, StartAngularVelocity);
    }
}

void URRTwoAngleInterpolationComponent::SetTargetPoint(float EndAngle, float EndAngularVelocity)
{
    if (AngleInterpolation)
    {
        AngleInterpolation->SetTargetPoint(EndAngle, EndAngularVelocity);
    }
}

void URRTwoAngleInterpolationComponent::SetConstraints(float MaxAngularAcceleration, float MaxAngularVelocity, float MaxAngularDeceleration)
{
    if (AngleInterpolation)
    {
        AngleInterpolation->SetConstraints(MaxAngularAcceleration, MaxAngularVelocity, MaxAngularDeceleration);
    }
}

void URRTwoAngleInterpolationComponent::Initialize(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                                                  float StartTime, float StartAngularVelocity, float EndAngularVelocity, float MaxAngularDeceleration)
{
    if (AngleInterpolation)
    {
        AngleInterpolation->Initialize(StartAngle, EndAngle, MaxAngularAcceleration, MaxAngularVelocity,
                                      StartTime, StartAngularVelocity, EndAngularVelocity, MaxAngularDeceleration);
    }
}

float URRTwoAngleInterpolationComponent::CalculateTrajectory()
{
    if (AngleInterpolation)
    {
        return AngleInterpolation->CalculateTrajectory();
    }
    return -1.0f;
}

float URRTwoAngleInterpolationComponent::CalculateTrajectoryWithParams(float StartAngle, float EndAngle, float MaxAngularAcceleration, float MaxAngularVelocity,
                                                                       float StartTime, float StartAngularVelocity, float EndAngularVelocity, float MaxAngularDeceleration)
{
    if (AngleInterpolation)
    {
        return AngleInterpolation->CalculateTrajectoryWithParams(StartAngle, EndAngle, MaxAngularAcceleration, MaxAngularVelocity,
                                                                 StartTime, StartAngularVelocity, EndAngularVelocity, MaxAngularDeceleration);
    }
    return -1.0f;
}

FTrajectoryPoint URRTwoAngleInterpolationComponent::GetPointAtTime(float Time) const
{
    if (AngleInterpolation)
    {
        return AngleInterpolation->GetPointAtTime(Time);
    }
    return FTrajectoryPoint();
}

bool URRTwoAngleInterpolationComponent::IsInitialized() const
{
    if (AngleInterpolation)
    {
        return AngleInterpolation->IsInitialized();
    }
    return false;
}

// URRTwoPointInterpolationLibrary Implementation

float URRTwoPointInterpolationLibrary::CalculatePositionTrajectoryDuration(float StartPosition, float EndPosition,
                                                                          float MaxAcceleration, float MaxVelocity,
                                                                          float StartTime, float StartVelocity,
                                                                          float EndVelocity, float MaxDeceleration)
{
    try 
    {
        TwoPointInterpolation interpolator(false); // Debug disabled
        
        // Convert UE units to library units
        double StartPositionM = static_cast<double>(StartPosition) / 100.0;
        double EndPositionM = static_cast<double>(EndPosition) / 100.0;
        double MaxAccelerationM = static_cast<double>(MaxAcceleration) / 100.0;
        double MaxVelocityM = static_cast<double>(MaxVelocity) / 100.0;
        double StartVelocityM = static_cast<double>(StartVelocity) / 100.0;
        double EndVelocityM = static_cast<double>(EndVelocity) / 100.0;
        double MaxDecelerationM = static_cast<double>(MaxDeceleration) / 100.0;
        
        interpolator.init(StartPositionM, EndPositionM, MaxAccelerationM, MaxVelocityM,
                         static_cast<double>(StartTime), StartVelocityM, EndVelocityM, MaxDecelerationM);
        
        double duration = interpolator.calcTrajectory();
        return static_cast<float>(duration);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("CalculatePositionTrajectoryDuration failed: %s"), UTF8_TO_TCHAR(e.what()));
        return -1.0f;
    }
}

float URRTwoPointInterpolationLibrary::CalculateAngleTrajectoryDuration(float StartAngle, float EndAngle,
                                                                       float MaxAngularAcceleration, float MaxAngularVelocity,
                                                                       float StartTime, float StartAngularVelocity,
                                                                       float EndAngularVelocity, float MaxAngularDeceleration)
{
    try 
    {
        // Convert degrees to radians for the library
        double StartAngleRad = FMath::DegreesToRadians(static_cast<double>(StartAngle));
        double EndAngleRad = FMath::DegreesToRadians(static_cast<double>(EndAngle));
        double MaxAccelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularAcceleration));
        double MaxVelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularVelocity));
        double StartVelRad = FMath::DegreesToRadians(static_cast<double>(StartAngularVelocity));
        double EndVelRad = FMath::DegreesToRadians(static_cast<double>(EndAngularVelocity));
        double DecelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularDeceleration));
        
        TwoAngleInterpolation interpolator(false); // Debug disabled
        interpolator.init(StartAngleRad, EndAngleRad, MaxAccelRad, MaxVelRad,
                         static_cast<double>(StartTime), StartVelRad, EndVelRad, DecelRad);
        
        double duration = interpolator.TwoPointInterpolation::calcTrajectory();
        return static_cast<float>(duration);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("CalculateAngleTrajectoryDuration failed: %s"), UTF8_TO_TCHAR(e.what()));
        return -1.0f;
    }
}

FTrajectoryPoint URRTwoPointInterpolationLibrary::GetPositionTrajectoryPointAtTime(float StartPosition, float EndPosition,
                                                                                  float MaxAcceleration, float MaxVelocity, float Time,
                                                                                  float StartTime, float StartVelocity,
                                                                                  float EndVelocity, float MaxDeceleration)
{
    try 
    {
        TwoPointInterpolation interpolator(false); // Debug disabled
        
        // Convert UE units to library units
        double StartPositionM = static_cast<double>(StartPosition) / 100.0;
        double EndPositionM = static_cast<double>(EndPosition) / 100.0;
        double MaxAccelerationM = static_cast<double>(MaxAcceleration) / 100.0;
        double MaxVelocityM = static_cast<double>(MaxVelocity) / 100.0;
        double StartVelocityM = static_cast<double>(StartVelocity) / 100.0;
        double EndVelocityM = static_cast<double>(EndVelocity) / 100.0;
        double MaxDecelerationM = static_cast<double>(MaxDeceleration) / 100.0;
        
        interpolator.init(StartPositionM, EndPositionM, MaxAccelerationM, MaxVelocityM,
                         static_cast<double>(StartTime), StartVelocityM, EndVelocityM, MaxDecelerationM);
        
        interpolator.calcTrajectory();
        std::vector<double> result = interpolator.getPoint(static_cast<double>(Time));
        
        if (result.size() >= 3)
        {
            // Convert library units back to UE units
            float PositionCm = static_cast<float>(result[0] * 100.0);
            float VelocityCm = static_cast<float>(result[1] * 100.0);
            float AccelerationCm = static_cast<float>(result[2] * 100.0);
            
            return FTrajectoryPoint(PositionCm, VelocityCm, AccelerationCm);
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("getPoint returned insufficient data"));
            return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
        }
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("GetPositionTrajectoryPointAtTime failed: %s"), UTF8_TO_TCHAR(e.what()));
        return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
    }
}

FTrajectoryPoint URRTwoPointInterpolationLibrary::GetAngleTrajectoryPointAtTime(float StartAngle, float EndAngle,
                                                                               float MaxAngularAcceleration, float MaxAngularVelocity, float Time,
                                                                               float StartTime, float StartAngularVelocity,
                                                                               float EndAngularVelocity, float MaxAngularDeceleration,
                                                                               bool bNormalizeOutput)
{
    try 
    {
        // Convert degrees to radians for the library
        double StartAngleRad = FMath::DegreesToRadians(static_cast<double>(StartAngle));
        double EndAngleRad = FMath::DegreesToRadians(static_cast<double>(EndAngle));
        double MaxAccelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularAcceleration));
        double MaxVelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularVelocity));
        double StartVelRad = FMath::DegreesToRadians(static_cast<double>(StartAngularVelocity));
        double EndVelRad = FMath::DegreesToRadians(static_cast<double>(EndAngularVelocity));
        double DecelRad = FMath::DegreesToRadians(static_cast<double>(MaxAngularDeceleration));
        
        TwoAngleInterpolation interpolator(false); // Debug disabled
        interpolator.init(StartAngleRad, EndAngleRad, MaxAccelRad, MaxVelRad,
                         static_cast<double>(StartTime), StartVelRad, EndVelRad, DecelRad);
        
        interpolator.TwoPointInterpolation::calcTrajectory();
        std::vector<double> result = interpolator.getPoint(static_cast<double>(Time), bNormalizeOutput);
        
        if (result.size() >= 3)
        {
            // Convert radians back to degrees
            float PositionDeg = FMath::RadiansToDegrees(static_cast<float>(result[0]));
            float VelocityDeg = FMath::RadiansToDegrees(static_cast<float>(result[1]));
            float AccelerationDeg = FMath::RadiansToDegrees(static_cast<float>(result[2]));
            
            return FTrajectoryPoint(PositionDeg, VelocityDeg, AccelerationDeg);
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("getPoint returned insufficient data for angle"));
            return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
        }
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("GetAngleTrajectoryPointAtTime failed: %s"), UTF8_TO_TCHAR(e.what()));
        return FTrajectoryPoint(0.0f, 0.0f, 0.0f);
    }
}

bool URRTwoPointInterpolationLibrary::ValidateTrajectoryParameters(float MaxAcceleration, float MaxVelocity, float MaxDeceleration)
{
    if (MaxAcceleration <= 0.0f)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Invalid MaxAcceleration: %.3f (must be positive)"), MaxAcceleration);
        return false;
    }
    
    if (MaxVelocity <= 0.0f)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Invalid MaxVelocity: %.3f (must be positive)"), MaxVelocity);
        return false;
    }
    
    if (MaxDeceleration > 0.0f && MaxDeceleration <= 0.0f)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Invalid MaxDeceleration: %.3f (must be positive if specified)"), MaxDeceleration);
        return false;
    }
    
    return true;
}