// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2Spawnable.h"

// rclUE
#include "Srvs/ROS2SpawnEntitySrv.h"

void UROS2Spawnable::InitializeParameters(const FROSSpawnEntity_Request& InRequest)
{
    SetName(InRequest.state_name);
    UE_LOG(LogTemp, Warning, TEXT("Pruning / from recieved namespace %s, namespace in UE4 will be set as: %s"), *InRequest.robot_namespace, *InRequest.robot_namespace.Replace(TEXT("/"),TEXT("")));
    SetNamespace(InRequest.robot_namespace.Replace(TEXT("/"),TEXT("")));
}

void UROS2Spawnable::SetName(const FString& InName)
{
    ActorName = InName;
}

void UROS2Spawnable::SetNamespace(const FString& InNamespace)
{
    ActorNamespace = InNamespace;
}

FString UROS2Spawnable::GetName()
{
    return ActorName;
}

FString UROS2Spawnable::GetNamespace()
{
    return ActorNamespace;
}
