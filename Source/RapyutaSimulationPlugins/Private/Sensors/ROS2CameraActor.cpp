// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/ROS2CameraActor.h"

AROS2CameraActor::AROS2CameraActor()
{
	
	// Setup camera defaults
	CameraComponent = CreateDefaultSubobject<UROS2CameraComponent>(TEXT("CameraComponent"));
	CameraComponent->CameraComponent->FieldOfView = 90.0f;
	CameraComponent->CameraComponent->bConstrainAspectRatio = true;
	CameraComponent->CameraComponent->AspectRatio = 1.777778f;
	CameraComponent->CameraComponent->PostProcessBlendWeight = 1.0f;
	
	RootComponent = CameraComponent;
}

void AROS2CameraActor::BeginPlay()
{
    Super::BeginPlay();
	
	// Node initialize
    Node = GetWorld()->SpawnActor<AROS2Node>();
	Node->Name =
        NodeName.IsEmpty() ? FString::Printf(TEXT("%s_ROS2CameraNode"), *(GetName())) : NodeName;
	Node->Namespace = NodeNamespace;
    Node->Init();
	Node->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
    
	CameraComponent->InitalizeWithROS2(Node);
}
 