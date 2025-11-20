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
        Interpolator.setInitial(0.0, static_cast<double>(StartPosition), static_cast<double>(StartVelocity));
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Start point set: Position=%.3f, Velocity=%.3f"), StartPosition, StartVelocity);
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
        Interpolator.setPoint(static_cast<double>(EndPosition), static_cast<double>(EndVelocity));
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Target point set: Position=%.3f, Velocity=%.3f"), EndPosition, EndVelocity);
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
        Interpolator.setConstraints(static_cast<double>(MaxAcceleration), static_cast<double>(MaxVelocity), 
                                   static_cast<double>(MaxDeceleration));
        UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Constraints set: MaxAccel=%.3f, MaxVel=%.3f, MaxDecel=%.3f"), 
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
        Interpolator.init(static_cast<double>(StartPosition), static_cast<double>(EndPosition), 
                         static_cast<double>(MaxAcceleration), static_cast<double>(MaxVelocity),
                         static_cast<double>(StartTime), static_cast<double>(StartVelocity), 
                         static_cast<double>(EndVelocity), static_cast<double>(MaxDeceleration));
        
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
                                                             float StartTime, float StartVelocity, float EndVelocity, float MaxDeceleration)
{
    try 
    {
        Interpolator.init(static_cast<double>(StartPosition), static_cast<double>(EndPosition), 
                         static_cast<double>(MaxAcceleration), static_cast<double>(MaxVelocity),
                         static_cast<double>(StartTime), static_cast<double>(StartVelocity), 
                         static_cast<double>(EndVelocity), static_cast<double>(MaxDeceleration));
        
        double duration = Interpolator.calcTrajectory();
        return static_cast<float>(duration);
    }
    catch (const std::exception& e)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("CalculateTrajectoryWithParams failed: %s"), UTF8_TO_TCHAR(e.what()));
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
            return FTrajectoryPoint(static_cast<float>(result[0]), static_cast<float>(result[1]), static_cast<float>(result[2]));
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