// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRBaseLidarComponent.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2LidarPublisher.h"

DEFINE_LOG_CATEGORY(LogROS2Sensor);

URRBaseLidarComponent::URRBaseLidarComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    BWithNoise = true;
}

void URRBaseLidarComponent::BeginPlay()
{
    Super::BeginPlay();
    GaussianRNGPosition = std::normal_distribution<>{PositionalNoiseMean, PositionalNoiseVariance};
    GaussianRNGIntensity = std::normal_distribution<>{IntensityNoiseMean, IntensityNoiseVariance};
}

void URRBaseLidarComponent::InitLidar(AROS2Node* InROS2Node, const FString& InTopicName)
{
    // Only need to initialize once
    if (nullptr == LidarPublisher)
    {
        // Instantiate Lidar publisher
        LidarPublisher = NewObject<URRROS2LidarPublisher>(this, *FString::Printf(TEXT("%sLidarPublisher"), *GetName()));
        LidarPublisher->LidarComponent = this;

        // Init [LidarPublisher] info
        LidarPublisher->PublicationFrequencyHz = ScanFrequency;
        LidarPublisher->MsgClass = LidarMsgClass;
    }

    // The publisher could have been garbaged for some reason
    if (IsValid(LidarPublisher))
    {
        // Update [LidarPublisher]'s topic name
        if (false == InTopicName.IsEmpty())
        {
            LidarPublisher->TopicName = InTopicName;
        }

        // Register [LidarPublisher] to the new ROS2 node
        LidarPublisher->InitializeWithROS2(InROS2Node);
    }

    // Start scanning the surroundings
    Run();
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
