/**
 * @file RRROS2Utils.h
 * @brief ROS2 Utils
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

DECLARE_DELEGATE_OneParam(FOnROS2MessagePublished, const UROS2GenericMsg*);

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

    template<typename T>
    static void SubscribeToTopic(AROS2Node* InROS2Node,
                                 T* InUserObject,
                                 const FString& InTopicName,
                                 const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                                 typename FOnROS2MessagePublished::TUObjectMethodDelegate<T>::FMethodPtr InCallback)
    {
        FSubscriptionCallback cb;
        cb.BindDynamic(InUserObject, InCallback);
        InROS2Node->AddSubscription(InTopicName, InMsgClass, cb);
    }
};
