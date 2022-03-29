// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/ROS2Spawnable.h"

// rclUE
#include "Srvs/ROS2SpawnEntitySrv.h"

void UROS2Spawnable::InitializeParameters(const FROSSpawnEntityRequest& InRequest)
{
    SetName(InRequest.StateName);
    UE_LOG(LogTemp, Warning, TEXT("Pruning / from recieved namespace %s, namespace in UE4 will be set as: %s"), *InRequest.RobotNamespace, *InRequest.RobotNamespace.Replace(TEXT("/"),TEXT("")));
    SetNamespace(InRequest.RobotNamespace.Replace(TEXT("/"),TEXT("")));
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
