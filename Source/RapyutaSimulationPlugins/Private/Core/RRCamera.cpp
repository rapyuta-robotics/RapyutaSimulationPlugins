// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRCamera.h"

// UE
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

// RapyutaSimInternal
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRMathUtils.h"
#include "Core/RRMeshActor.h"
#include "Core/RRUObjectUtils.h"

ARRCamera::ARRCamera()
{
    CameraComponent = URRUObjectUtils::CreateAndAttachChildComponent<UCameraComponent>(
        this, FString::Printf(TEXT("%s_CameraComponent"), *GetName()));
    SetRootComponent(CameraComponent);
}

void ARRCamera::PrintSimConfig() const
{
    Super::PrintSimConfig();

    UE_LOG(LogRapyutaCore, Display, TEXT("CAMERA CONFIG -----------------------------"));
    CameraProperties.PrintSelf();
}

bool ARRCamera::Initialize()
{
    if (false == Super::Initialize())
    {
        return false;
    }
    CameraComponent->FieldOfView = URRMathUtils::GetRandomFloatInRange(CameraProperties.HFoVRangeInDegree);
    return true;
}

void ARRCamera::LookAt(const FVector& InTargetLocation)
{
    CameraComponent->SetWorldRotation((InTargetLocation - GetActorLocation()).ToOrientationQuat());
}

void ARRCamera::RandomizeFoV()
{
    CameraComponent->FieldOfView = URRMathUtils::GetRandomFloatInRange(CameraProperties.HFoVRangeInDegree);
}

void ARRCamera::RandomizePose(bool bIsRandomLocationOnly)
{
    const FVector sceneInstanceLocation = URRCoreUtils::GetSceneInstanceLocation(SceneInstanceId);
    const FVector randomLocation = URRMathUtils::GetRandomSphericalPosition(
        sceneInstanceLocation, CameraProperties.DistanceRangeInCm, CameraProperties.HeightRangeInCm);

    if (bIsRandomLocationOnly)
    {
        SetActorLocation(randomLocation);
    }
    else
    {
        const FRotator randomRotation = (-randomLocation).ToOrientationRotator();
        SetActorTransform(FTransform(randomRotation, randomLocation));
    }
}

float ARRCamera::GetDistanceToFloor() const
{
    check(IsValid(ActorCommon->MainFloor));
    return FVector::Dist(CameraComponent->GetComponentLocation(), ActorCommon->MainFloor->GetActorLocation());
};
