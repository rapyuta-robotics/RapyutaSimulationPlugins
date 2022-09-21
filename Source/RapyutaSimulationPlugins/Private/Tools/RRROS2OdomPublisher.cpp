// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2OdomPublisher.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RobotVehicle.h"

URRROS2OdomPublisher::URRROS2OdomPublisher()
{
    MsgClass = UROS2OdometryMsg::StaticClass();
    TopicName = TEXT("odom");
    PublicationFrequencyHz = 30;
}

void URRROS2OdomPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    // [URRROS2OdomPublisher] must have been already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::KeepLast);

    // Init TF
    InitializeTFWithROS2(InROS2Node);
}

void URRROS2OdomPublisher::InitializeTFWithROS2(AROS2Node* InROS2Node)
{
    if (bPublishOdomTf)
    {
        if (nullptr == TFPublisher)
        {
            TFPublisher = NewObject<URRROS2TFPublisher>(this);
            TFPublisher->SetupUpdateCallback();
            TFPublisher->PublicationFrequencyHz = PublicationFrequencyHz;
        }
        TFPublisher->InitializeWithROS2(InROS2Node);
    }
}

void URRROS2OdomPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    FROSOdometry odomData;
    bool bIsOdomDataValid = GetOdomData(odomData);
    if (bIsOdomDataValid)
    {
        CastChecked<UROS2OdometryMsg>(InMessage)->SetMsg(odomData);
    }
}

bool URRROS2OdomPublisher::GetOdomData(FROSOdometry& OutOdomData) const
{
    const URobotVehicleMovementComponent* moveComponent =
        RobotVehicle.IsValid() ? RobotVehicle.Get()->RobotVehicleMoveComponent : nullptr;
    if (moveComponent)
    {
        
        OutOdomData = URRConversionUtils::OdomUEToROS(moveComponent->OdomData);
        if (bAppendNodeNamespace)
        {
            OutOdomData.HeaderFrameId = URRGeneralUtils::ComposeROSFullFrameId(OwnerNode->Namespace, *OutOdomData.HeaderFrameId);
            OutOdomData.ChildFrameId = URRGeneralUtils::ComposeROSFullFrameId(OwnerNode->Namespace, *OutOdomData.ChildFrameId);
        }
        
        if (bPublishOdomTf && TFPublisher)
        {
            TFPublisher->TF = moveComponent->GetOdomTF();
            TFPublisher->FrameId = OutOdomData.HeaderFrameId;
            TFPublisher->ChildFrameId = OutOdomData.ChildFrameId;
        }
        
        return true;
    }
    else
    {
        return false;
    }
}

void URRROS2OdomPublisher::RevokeUpdateCallback()
{
    Super::RevokeUpdateCallback();
    if (bPublishOdomTf && TFPublisher)
    {
        TFPublisher->RevokeUpdateCallback();
    }
}
