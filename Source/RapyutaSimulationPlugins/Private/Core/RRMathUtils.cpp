// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRMathUtils.h"

#include "RapyutaSimulationPlugins.h"
#include "logUtilities.h"

FRandomStream URRMathUtils::RandomStream = FRandomStream();

void URRMathUtils::InitializeRandomStream()
{
    RandomStream.GenerateNewSeed();
    UE_LOG_WITH_INFO(
        LogRapyutaCore, Display, TEXT("RRSim Random generator was initialized with seed: %d"), RandomStream.GetCurrentSeed());
}

FVector URRMathUtils::GetRandomSphericalPosition(const FVector& InCenter,
                                                 const FVector2f& InDistanceRange,
                                                 const FVector2f& InHeightRange)
{
    // Spherical coordinate (r, θ, φ)
    const float randRadialDistance = URRMathUtils::GetRandomFloatInRange(InDistanceRange);
    const float randHeight = URRMathUtils::GetRandomFloatInRange(InHeightRange);

    // Azimuthal angle θ (Azimuth)
    const float randAzimuthalAngle = GetRandomFloatInRange(-PI, PI);

    return InCenter + FVector(randRadialDistance * FMath::Cos(randAzimuthalAngle),
                              randRadialDistance * FMath::Sin(randAzimuthalAngle),
                              randHeight);
}
