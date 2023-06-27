// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2BaseSensorComponent.h"

DEFINE_LOG_CATEGORY(LogROS2Sensor);

URRROS2BaseSensorComponent::URRROS2BaseSensorComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void URRROS2BaseSensorComponent::InitalizeWithROS2(UROS2NodeComponent* InROS2Node,
                                                   const FString& InPublisherName,
                                                   const FString& InTopicName,
                                                   const UROS2QoS InQoS)
{
    CreatePublisher(InPublisherName);
    PreInitializePublisher(InROS2Node, InTopicName);
    InitializePublisher(InROS2Node, InQoS);

    // Start getting sensor data
    Run();
}

void URRROS2BaseSensorComponent::CreatePublisher(const FString& InPublisherName)
{
    // Init [SensorPublisher] info
    if (nullptr == SensorPublisher)
    {
        verify(SensorPublisherClass);
        // Instantiate publisher
        SensorPublisher = NewObject<URRROS2BaseSensorPublisher>(
            this,
            SensorPublisherClass,
            InPublisherName.IsEmpty() ? *FString::Printf(TEXT("%sSensorPublisher"), *GetName()) : *InPublisherName);
        SensorPublisher->DataSourceComponent = this;
    }
}

void URRROS2BaseSensorComponent::PreInitializePublisher(UROS2NodeComponent* InROS2Node, const FString& InTopicName)
{
    if (IsValid(SensorPublisher))
    {
        // Overwrite MsgClass if it is not default value.
        if (MsgClass != UROS2GenericMsg::StaticClass())
        {
            SensorPublisher->MsgClass = MsgClass;
        }
        SensorPublisher->PublicationFrequencyHz = PublicationFrequencyHz;

        // Update [SensorPublisher]'s topic name
        SensorPublisher->TopicName = InTopicName.IsEmpty() ? TopicName : InTopicName;

        if (bAppendNodeNamespace)
        {
            FrameId = URRGeneralUtils::ComposeROSFullFrameId(InROS2Node->Namespace, *FrameId);
        }
    }
}

void URRROS2BaseSensorComponent::InitializePublisher(UROS2NodeComponent* InROS2Node, const UROS2QoS InQoS)
{
    if (IsValid(SensorPublisher))
    {
        SensorPublisher->InitializeWithROS2(InROS2Node);
        SensorPublisher->QoS = InQoS;
        SensorPublisher->Init();
    }
}

void URRROS2BaseSensorComponent::Run()
{
    GetWorld()->GetTimerManager().SetTimer(
        TimerHandle, this, &URRROS2BaseSensorComponent::SensorUpdate, 1.f / static_cast<float>(PublicationFrequencyHz), true);
}

void URRROS2BaseSensorComponent::Stop()
{
    GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
}
