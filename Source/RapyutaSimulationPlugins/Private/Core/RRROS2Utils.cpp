// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Core/RRROS2Utils.h"

UROS2Publisher* URRROS2Utils::CreatePublisher(UObject* InOwner,
                                              const FString& InTopicName,
                                              const TSubclassOf<UROS2Publisher>& InPublisherClass,
                                              const TSubclassOf<UROS2GenericMsg>& InMsgClass,
                                              int32 InPubFrequency)
{
    UROS2Publisher* publisher = NewObject<UROS2Publisher>(InOwner, InPublisherClass);
    publisher->MsgClass = InMsgClass;
    publisher->TopicName = InTopicName;
    publisher->PublicationFrequencyHz = InPubFrequency;
    publisher->SetupUpdateCallback();
    return publisher;
}
