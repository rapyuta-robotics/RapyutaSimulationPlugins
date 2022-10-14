// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2Spawnable.h"

// rclUE
#include "Srvs/ROS2SpawnEntity.h"

void UROS2Spawnable::OnComponentCreated()
{
    Super::OnComponentCreated();
    SetIsReplicated(true);
}

void UROS2Spawnable::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(UROS2Spawnable, ActorName);
    DOREPLIFETIME(UROS2Spawnable, ActorNamespace);
    DOREPLIFETIME(UROS2Spawnable, ActorTags);
    DOREPLIFETIME(UROS2Spawnable, NetworkPlayerId);
}

void UROS2Spawnable::InitializeParameters(const FROSSpawnEntityReq& InRequest)
{
    SetName(InRequest.State.Name);
    UE_LOG(LogTemp,
           Warning,
           TEXT("Pruning / from recieved namespace %s, namespace in UE4 will be set as: %s"),
           *InRequest.RobotNamespace,
           *InRequest.RobotNamespace.Replace(TEXT("/"), TEXT("")));
    SetNamespace(InRequest.RobotNamespace.Replace(TEXT("/"), TEXT("")));
}

void UROS2Spawnable::SetName(const FString& InName)
{
    ActorName = InName;
}

void UROS2Spawnable::SetNamespace(const FString& InNamespace)
{
    ActorNamespace = InNamespace;
}

void UROS2Spawnable::AddTag(const FString& InTag)
{
    ActorTags.Emplace(InTag);
}

FString UROS2Spawnable::GetName() const
{
    return ActorName;
}

FString UROS2Spawnable::GetNamespace() const
{
    return ActorNamespace;
}

int32 UROS2Spawnable::GetNetworkPlayerId() const
{
    return NetworkPlayerId;
}

void UROS2Spawnable::SetNetworkPlayerId(const int32 InNetworkPlayerId)
{
    NetworkPlayerId = InNetworkPlayerId;
}
