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
    SetDefaultDelegates();    //use UpdateMessage as update delegate
}

bool URRROS2OdomPublisher::InitializeWithROS2(UROS2NodeComponent* InROS2Node)
{
    bool res = Super::InitializeWithROS2(InROS2Node);

    if (res)
    {
        // Init TF
        InitializeTFWithROS2(InROS2Node);
    }

    return res;
}

void URRROS2OdomPublisher::InitializeTFWithROS2(UROS2NodeComponent* InROS2Node)
{
    if (bPublishOdomTf && nullptr == TFPublisher)
    {
        TFPublisher = CastChecked<URRROS2TFPublisher>(InROS2Node->CreatePublisherWithClass(URRROS2TFPublisher::StaticClass()));
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
