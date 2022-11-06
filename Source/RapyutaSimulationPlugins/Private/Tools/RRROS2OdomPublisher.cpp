// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2OdomPublisher.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RobotVehicle.h"

URRROS2OdomPublisher::URRROS2OdomPublisher()
{
    MsgClass = UROS2OdomMsg::StaticClass();
    TopicName = TEXT("odom");
    PublicationFrequencyHz = 30;
    QoS = UROS2QoS::KeepLast;
}

void URRROS2OdomPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

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
    FROSOdom odomData;
    bool bIsOdomDataValid = GetOdomData(odomData);
    if (bIsOdomDataValid)
    {
        CastChecked<UROS2OdomMsg>(InMessage)->SetMsg(odomData);
    }
}

bool URRROS2OdomPublisher::GetOdomData(FROSOdom& OutOdomData) const
{
    const URobotVehicleMovementComponent* moveComponent =
        RobotVehicle.IsValid() ? RobotVehicle.Get()->RobotVehicleMoveComponent : nullptr;
    if (moveComponent)
    {
        
        OutOdomData = URRConversionUtils::OdomUEToROS(moveComponent->OdomData);
        if (bAppendNodeNamespace)
        {
            OutOdomData.Header.FrameId = URRGeneralUtils::ComposeROSFullFrameId(OwnerNode->Namespace, *OutOdomData.Header.FrameId);
            OutOdomData.ChildFrameId = URRGeneralUtils::ComposeROSFullFrameId(OwnerNode->Namespace, *OutOdomData.ChildFrameId);
        }
        
        if (bPublishOdomTf && TFPublisher)
        {
            TFPublisher->TF = moveComponent->GetOdomTF();
            TFPublisher->FrameId = OutOdomData.Header.FrameId;
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
