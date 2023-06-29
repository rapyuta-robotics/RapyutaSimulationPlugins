// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2CameraActor.h"

ARRROS2CameraActor::ARRROS2CameraActor()
{
    // Setup camera defaults
    CameraComponent = CreateDefaultSubobject<URRROS2CameraComponent>(TEXT("CameraComponent"));
    CameraComponent->CameraComponent->FieldOfView = 90.0f;
    CameraComponent->CameraComponent->bConstrainAspectRatio = true;
    CameraComponent->CameraComponent->AspectRatio = 1.777778f;
    CameraComponent->CameraComponent->PostProcessBlendWeight = 1.0f;

    RootComponent = CameraComponent;
}

void ARRROS2CameraActor::BeginPlay()
{
    Super::BeginPlay();

    // Node initialize
    Node = NewObject<UROS2NodeComponent>(this);
    Node->Name = NodeName.IsEmpty() ? FString::Printf(TEXT("%s_RRROS2CameraNode"), *(GetName())) : NodeName;
    Node->Namespace = NodeNamespace;
    Node->Init();

    CameraComponent->InitalizeWithROS2(Node);
}
