// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/RRGhostPlayerPawn.h"

// UE
#include "Components/SphereComponent.h"

ARRGhostPlayerPawn::ARRGhostPlayerPawn()
{
    GetMeshComponent()->SetStaticMesh(nullptr);
    GetCollisionComponent()->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
};
