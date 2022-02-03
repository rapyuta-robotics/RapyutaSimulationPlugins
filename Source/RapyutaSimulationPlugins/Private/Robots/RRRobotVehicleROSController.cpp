// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robots/RRRobotVehicleROSController.h"

// rclUE
#include "Msgs/ROS2TwistMsg.h"
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Robots/RobotVehicle.h"
#include "Tools/UEUtilities.h"

void ARRRobotVehicleROSController::SubscribeToMovementCommandTopic(const FString& InTopicName)
{
    // Subscription with callback to enqueue vehicle spawn info.
    if (ensure(IsValid(RobotROS2Node)))
    {
        FSubscriptionCallback cb;
        cb.BindDynamic(this, &ARRRobotVehicleROSController::MovementCallback);
        RobotROS2Node->AddSubscription(InTopicName, UROS2TwistMsg::StaticClass(), cb);
    }
}

void ARRRobotVehicleROSController::MovementCallback(const UROS2GenericMsg* Msg)
{
    const UROS2TwistMsg* twistMsg = Cast<UROS2TwistMsg>(Msg);
    if (IsValid(twistMsg))
    {
        // TODO refactoring will be needed to put units and system of reference conversions in a consistent location
        // probably should not stay in msg though
        FROSTwist twist;
        twistMsg->GetMsg(twist);
        const FVector linear(ConversionUtils::VectorROSToUE(twist.linear));
        const FVector angular(ConversionUtils::RotationROSToUE(twist.angular));

        // (Note) In this callback, which could be invoked from a ROS working thread,
        // the ROSController itself (this) could have been garbage collected,
        // thus any direct referencing to its member in this GameThread lambda needs to be verified.
        AsyncTask(ENamedThreads::GameThread,
                  [linear, angular, vehicle = CastChecked<ARobotVehicle>(GetPawn())]
                  {
                      if (IsValid(vehicle))
                      {
                          vehicle->SetLinearVel(linear);
                          vehicle->SetAngularVel(angular);
                      }
                  });
    }
}
