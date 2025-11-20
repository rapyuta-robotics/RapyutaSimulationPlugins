/**
 * @file RRTwoPointInterpolationLibrary.cpp
 * @brief Blueprint function library for TwoPointInterpolation utilities implementation
 * @copyright Copyright 2025 Rapyuta Robotics Co., Ltd.
 */

#include "Tools/RRTwoPointInterpolationLibrary.h"

// UE
#include "Engine/Engine.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"

// ThirdParty
#include "two_points_interpolation_constant_acc.hpp"

float URRTwoPointInterpolationLibrary::CalculatePositionTrajectoryDuration(float StartPosition, float EndPosition,
                                                                          float MaxAcceleration, float MaxVelocity,
                                                                          float StartTime, float StartVelocity,
                                                                          float EndVelocity, float MaxDeceleration)
{
    try 
    {
        TwoPointInterpolation interpolator(false); // Debug disabled
        interpolator.init(static_cast<double>(StartPosition), static_cast<double>(EndPosition),
                         static_cast<double>(MaxAcceleration), static_cast<double>(MaxVelocity),
                         static_cast<double>(StartTime), static_cast<double>(StartVelocity),
                         static_cast<double>(EndVelocity), static_cast<double>(MaxDeceleration));
        
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
        interpolator.init(static_cast<double>(StartPosition), static_cast<double>(EndPosition),
                         static_cast<double>(MaxAcceleration), static_cast<double>(MaxVelocity),
                         static_cast<double>(StartTime), static_cast<double>(StartVelocity),
                         static_cast<double>(EndVelocity), static_cast<double>(MaxDeceleration));
        
        interpolator.calcTrajectory();
        std::vector<double> result = interpolator.getPoint(static_cast<double>(Time));
        
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