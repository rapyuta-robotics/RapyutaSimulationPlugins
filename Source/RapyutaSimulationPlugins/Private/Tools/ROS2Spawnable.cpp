//
// Created by lng on 2021/10/13.
//

#include "Tools/ROS2Spawnable.h"

// rclUE
#include "Srvs/ROS2SpawnEntitySrv.h"

void UROS2Spawnable::InitializeParameters(FROSSpawnEntity_Request Request)
{
    SetName(Request.state_name);
    SetNamespace(Request.robot_namespace);
}

void UROS2Spawnable::SetName(FString Name)
{
    ActorName = Name;
}

void UROS2Spawnable::SetNamespace(FString Namespace)
{
    ActorNamespace = Namespace;
}

FString UROS2Spawnable::GetName()
{
    return ActorName;
}

FString UROS2Spawnable::GetNamespace()
{
    return ActorNamespace;
}