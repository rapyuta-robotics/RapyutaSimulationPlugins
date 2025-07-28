// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRROS2ActorsRvizMarkerPublisher.h"

#include "Core/RRConversionUtils.h"
#include "Core/RRGeneralUtils.h"

URRROS2ActorsRvizMarkerPublisher::URRROS2ActorsRvizMarkerPublisher()
{
    ActorClass = APawn::StaticClass();
    TopicName = TEXT("rviz_marker");
    MsgClass = UROS2MarkerArrayMsg::StaticClass();
    PublicationFrequencyHz = 1;
    QoS = UROS2QoS::KeepLast;
    SetDefaultDelegates();    //use UpdateMessage as update delegate
}

void URRROS2ActorsRvizMarkerPublisher::AddTargetActor(AActor* InActor)
{
    if (false == Actors.Contains(InActor))
    {
        InActor->OnDestroyed.AddDynamic(this, &URRROS2ActorsRvizMarkerPublisher::OnTargetActorDestroyed);
        Actors.Add(InActor);
    }
}

void URRROS2ActorsRvizMarkerPublisher::OnTargetActorDestroyed(AActor* InActor)
{
    Actors.RemoveSwap(InActor);
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
        if (!IsValid(actor))
        {
            continue;
        }
        FROSMarker marker = BaseMarker;
        FTransform tf =
            URRConversionUtils::TransformUEToROS(URRGeneralUtils::GetRelativeTransform(ReferenceActor, actor->GetTransform()));
        marker.Ns = actor->GetName();
        marker.Pose.Position = tf.GetTranslation();
        marker.Pose.Orientation = tf.GetRotation();
        msg.Markers.Add(marker);
    }

    if (msg.Markers.Num() > 0)
    {
        CastChecked<UROS2MarkerArrayMsg>(InMessage)->SetMsg(msg);
    }
}
