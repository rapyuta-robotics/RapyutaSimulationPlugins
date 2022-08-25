// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/GhostPlayerPawn.h"

// UE
#include "Components/SphereComponent.h"

AGhostPlayerPawn::AGhostPlayerPawn()
{
    GetMeshComponent()->SetStaticMesh(nullptr);
    GetCollisionComponent()->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
};
