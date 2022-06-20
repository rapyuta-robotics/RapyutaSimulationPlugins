/**
 * @file RRROS2Utils.h
 * @brief ROS2 high-level utils
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// rclUE
#include "Msgs/ROS2GenericMsg.h"
#include "ROS2Node.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "RapyutaSimulationPlugins.h"

#include "RRROS2Utils.generated.h"

/**
 * @brief RR_ROS2_SUBSCRIBE_TO_TOPIC
 * FSubscriptionCallback is of dynamic delegate type to be serializable for BP use
 * FSubscriptionCallback::BindDynamic is a macro, instead of a function.
 * Thus InCallback can only be a direct UFUNCTION() method & cannot be used as typed param!
 */
#define RR_ROS2_SUBSCRIBE_TO_TOPIC(InROS2Node, InUserObject, InTopicName, InMsgClass, InCallback) \
    if (ensure(IsValid(InROS2Node)))                                                              \
    {                                                                                             \
        FSubscriptionCallback cb;                                                                 \
        cb.BindDynamic(InUserObject, InCallback);                                                 \
        InROS2Node->AddSubscription(InTopicName, InMsgClass, cb);                                 \
    }

/**
 * @brief ROS2 high-level utils
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRROS2Utils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    // Refer to rr_sootball's sootball_fast.model.yaml/sootball_gen2.model.yaml for [update_rate] as pub frequency
    static UROS2Publisher* CreatePublisher(UObject* InOwner,
                                           const FString& InTopicName,
                                           const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                           const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                                           int32 InPubFrequency);
};
