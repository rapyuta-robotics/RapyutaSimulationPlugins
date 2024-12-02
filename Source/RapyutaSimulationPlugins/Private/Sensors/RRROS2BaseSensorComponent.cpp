// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

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
    // NOTE: Here, [SensorPublisher] is expected to finish custom configuring before being added to [InROS2Node]'s Publishers and init.
    // Thus [UROS2NodeComponent::CreatePublisher()] is not used.

    // Create & Init [SensorPublisher]
    CreatePublisher(InPublisherName);
    PreInitializePublisher(InROS2Node, InTopicName);
    // [SensorPublisher] is added to [InROS2Node]'s Publishers here-in
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
        SensorPublisher->QoS = InQoS;
        // [InitializeWithROS2(InROS2Node) & SensorPublisher->Init()] are triggered here-in
        InROS2Node->AddPublisher(SensorPublisher);
    }
}

void URRROS2BaseSensorComponent::Run()
{
    GetWorld()->GetTimerManager().SetTimer(
        TimerHandle, this, &URRROS2BaseSensorComponent::SensorUpdate, 1.f / static_cast<float>(PublicationFrequencyHz), true);
}

void URRROS2BaseSensorComponent::Stop()
{
    if (SensorPublisher)
    {
        SensorPublisher->StopPublishTimer();
    }
    GetWorld()->GetTimerManager().ClearTimer(TimerHandle);
}

void URRROS2BaseSensorComponent::DestroyComponent(bool bPromoteChildren)
{
    Stop();
    if (SensorPublisher)
    {
        SensorPublisher->Destroy();
    }
    Super::DestroyComponent();
}
