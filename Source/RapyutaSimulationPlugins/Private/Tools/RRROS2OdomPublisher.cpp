// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2OdomPublisher.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RobotVehicle.h"

URRROS2OdomPublisher::URRROS2OdomPublisher()
{
    MsgClass = UROS2OdomMsg::StaticClass();
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
        TFPublisher = CastChecked<URRROS2TFPublisher>(
            InROS2Node->CreateLoopPublisherWithClass(TEXT("/tf"), URRROS2TFPublisher::StaticClass(), PublicationFrequencyHz));
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
    URRBaseOdomComponent* odomSource = Cast<URRBaseOdomComponent>(DataSourceComponent);
    if (odomSource)
    {
        OutOdomData = URRConversionUtils::OdomUEToROS(odomSource->OdomData);
        if (bAppendNodeNamespace)
        {
            OutOdomData.ChildFrameId = URRGeneralUtils::ComposeROSFullFrameId(OwnerNode->Namespace, *OutOdomData.ChildFrameId);
        }

        if (bPublishOdomTf && TFPublisher)
        {
            TFPublisher->TF = odomSource->GetOdomTF();
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
