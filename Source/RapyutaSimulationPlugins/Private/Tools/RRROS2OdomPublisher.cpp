// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2OdomPublisher.h"

// RapyutaSimulationPlugins
#include "Drives/RobotVehicleMovementComponent.h"
#include "Robots/RobotVehicle.h"

void URRROS2OdomPublisher::InitializeWithROS2(AROS2Node* InROS2Node)
{
    Super::InitializeWithROS2(InROS2Node);

    MsgClass = UROS2OdometryMsg::StaticClass();
    TopicName = TEXT("odom");
    PublicationFrequencyHz = 30;

    // [URRROS2OdomPublisher] must have been already registered to [InROS2Node] (in Super::) before being initialized
    Init(UROS2QoS::KeepLast);

    // Init TF
    InitializeTFWithROS2(InROS2Node);
}

void URRROS2OdomPublisher::InitializeTFWithROS2(AROS2Node* InROS2Node)
{
    if (nullptr == TFPublisher)
    {
        TFPublisher = NewObject<URRROS2TFPublisher>(this);
        TFPublisher->SetupUpdateCallback();
    }
    TFPublisher->InitializeWithROS2(InROS2Node);
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
    if (moveComponent && moveComponent->OdomData.IsValid())
    {
        // Also update TFPublisher's TF
        TFPublisher->TF = moveComponent->GetOdomTF();
        OutOdomData = ConversionUtils::OdomUEToROS(moveComponent->OdomData);
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
    TFPublisher->RevokeUpdateCallback();
}
