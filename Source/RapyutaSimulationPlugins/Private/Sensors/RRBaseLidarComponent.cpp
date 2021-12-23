// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRBaseLidarComponent.h"

DEFINE_LOG_CATEGORY(LogROS2Sensor);

URRBaseLidarComponent::URRBaseLidarComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    LidarPublisher = CreateDefaultSubobject<UROS2Publisher>(*FString::Printf(TEXT("%sLidarPublisher"), *GetName()));
    LidarPublisher->TopicName = TEXT("scan");
    LidarPublisher->PublicationFrequencyHz = ScanFrequency;
    LidarPublisher->MsgClass = LidarMsgClass;
}

void URRBaseLidarComponent::BeginPlay()
{
    Super::BeginPlay();
    GaussianRNGPosition = std::normal_distribution<>{PositionalNoiseMean, PositionalNoiseVariance};
    GaussianRNGIntensity = std::normal_distribution<>{IntensityNoiseMean, IntensityNoiseVariance};
}

void URRBaseLidarComponent::InitLidar(AROS2Node* InROS2Node, const FString& InTopicName)
{
    LidarPublisher->TopicName = InTopicName;
    LidarPublisher->PublicationFrequencyHz = ScanFrequency;
    LidarPublisher->OwnerNode = InROS2Node;

    InitToNode(InROS2Node);
}

void URRBaseLidarComponent::GetData(TArray<FHitResult>& OutHits, float& OutTime)
{
    // what about the rest of the information?
    OutHits = RecordedHits;
    OutTime = TimeOfLastScan;
}

FLinearColor URRBaseLidarComponent::GetColorFromIntensity(const float InIntensity)
{
    float normalizedIntensity = (InIntensity - IntensityMin) / (IntensityMax - IntensityMin);
    return InterpolateColor(normalizedIntensity);
}

FLinearColor URRBaseLidarComponent::InterpolateColor(float InX)
{
    // this means that viz and data sent won't correspond, which should be ok
    InX = InX + WithNoise * GaussianRNGIntensity(Gen);
    return InX > .5f ? FLinearColor::LerpUsingHSV(ColorMid, ColorMax, 2 * InX - 1)
                     : FLinearColor::LerpUsingHSV(ColorMin, ColorMid, 2 * InX);
}

float URRBaseLidarComponent::IntensityFromDist(float InBaseIntensity, float InDistance)
{
    return InBaseIntensity * 1.3f * exp(-.1f * (pow(3.5f * InDistance, .6f))) / (1 + exp(-((3.5f * InDistance))));
}
