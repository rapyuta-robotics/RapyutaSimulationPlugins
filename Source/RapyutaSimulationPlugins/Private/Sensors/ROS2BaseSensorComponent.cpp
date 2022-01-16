// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/ROS2BaseSensorComponent.h"

DEFINE_LOG_CATEGORY(LogROS2Sensor);

UROS2BaseSensorComponent::UROS2BaseSensorComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UROS2BaseSensorComponent::InitalizeWithROS2(AROS2Node* InROS2Node, const FString& InTopicName, const TEnumAsByte<UROS2QoS> InQoS)
{

    CreatePublisher();
    PreInitializePublisher(InROS2Node, InTopicName);
    InitializePublisher(InROS2Node, InQoS);

    // Start getting sensor data
    Run();
}

void UROS2BaseSensorComponent::CreatePublisher()
{
    // Init [SensorPublisher] info
    if (nullptr == SensorPublisher)
    {
        // Instantiate publisher
        SensorPublisher = NewObject<UROS2Publisher>(this, *FString::Printf(TEXT("%sSensorPublisher"), *GetName()));
    }
}

void UROS2BaseSensorComponent::PreInitializePublisher(AROS2Node* InROS2Node, const FString& InTopicName)
{
    if (IsValid(SensorPublisher))
    {
        SensorPublisher->PublicationFrequencyHz = PublicationFrequencyHz;
        verify(SensorMsgClass);
        SensorPublisher->MsgClass = SensorMsgClass;

        // Update [SensorPublisher]'s topic name
        SensorPublisher->TopicName = InTopicName.IsEmpty() ? TopicName : InTopicName;
        verify(false == SensorPublisher->TopicName.IsEmpty());

        if (bAppendNodeNamespace)
        {
            FrameId = URRGeneralUtils::ComposeROSFullFrameId(InROS2Node->Namespace, *FrameId);
            verify(false == FrameId.IsEmpty());
        }
    }
}

void UROS2BaseSensorComponent::InitializePublisher(AROS2Node* InROS2Node, const TEnumAsByte<UROS2QoS> InQoS)
{
    if (IsValid(SensorPublisher))
    {
        InROS2Node->AddPublisher(SensorPublisher);
        SensorPublisher->Init(InQoS);
    }
}
