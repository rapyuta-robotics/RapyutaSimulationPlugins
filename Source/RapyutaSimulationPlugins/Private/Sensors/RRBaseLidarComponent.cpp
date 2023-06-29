// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRBaseLidarComponent.h"

URRBaseLidarComponent::URRBaseLidarComponent()
{
    BWithNoise = true;
    TopicName = TEXT("scan");
    FrameId = TEXT("base_scan");
}

void URRBaseLidarComponent::BeginPlay()
{
    Super::BeginPlay();
    GaussianRNGPosition = std::normal_distribution<>{PositionalNoiseMean, PositionalNoiseVariance};
    GaussianRNGIntensity = std::normal_distribution<>{IntensityNoiseMean, IntensityNoiseVariance};
}

void URRBaseLidarComponent::GetData(TArray<FHitResult>& OutHits, float& OutTime) const
{
    // what about the rest of the information?
    OutHits = RecordedHits;
    OutTime = TimeOfLastScan;
}

FLinearColor URRBaseLidarComponent::InterpColorFromIntensity(const float InIntensity)
{
    return InterpolateColor(FMath::GetRangePct(IntensityMin, IntensityMax, InIntensity));
}

FLinearColor URRBaseLidarComponent::InterpolateColor(float InX)
{
    // this means that viz and data sent won't correspond, which should be ok
    InX = InX + BWithNoise * GaussianRNGIntensity(Gen);
    return (InX > .5f) ? FLinearColor::LerpUsingHSV(ColorMid, ColorMax, 2 * InX - 1)
                       : FLinearColor::LerpUsingHSV(ColorMin, ColorMid, 2 * InX);
}

float URRBaseLidarComponent::GetIntensityFromDist(float InBaseIntensity, float InDistance)
{
    return InBaseIntensity * 1.3f * FMath::Exp(-.1f * (FMath::Pow(3.5f * InDistance, .6f))) /
           (1 + FMath::Exp(-((3.5f * InDistance))));
}
