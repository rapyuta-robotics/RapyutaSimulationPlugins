// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ActorsRvizMarkerPublisher.h"

URRROS2ActorsRvizMarkerPublisher::URRROS2ActorsRvizMarkerPublisher()
{
    ActorClass = APawn::StaticClass();
    TopicName = TEXT("rviz_marker");
    MsgClass = UROS2MarkerArrayMsg::StaticClass();
    PublicationFrequencyHz = 1;
    QoS = UROS2QoS::KeepLast;
    SetDefaultDelegates();    //use UpdateMessage as update delegate
}

void URRROS2ActorsRvizMarkerPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (bUpdateActorsList)
    {
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), ActorClass, Actors);
    }

    FROSMarkerArray msg;
    BaseMarker.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));
    for (AActor* actor : Actors)
    {
        FROSMarker marker = BaseMarker;
        FTransform tf =
            URRConversionUtils::TransformUEToROS(URRGeneralUtils::GetRelativeTransform(ReferenceActor, actor->GetTransform()));
        marker.Ns = actor->GetName();
        marker.Pose.Position = tf.GetTranslation();
        marker.Pose.Orientation = tf.GetRotation();
        msg.Markers.Add(marker);
    }

    CastChecked<UROS2MarkerArrayMsg>(InMessage)->SetMsg(msg);
}
