// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

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
    RandomizeFoV();
    return true;
}

void ARRCamera::LookAt(const FVector& InTargetLocation, bool bMoveToNearTarget)
{
    if (bMoveToNearTarget)
    {
        SetActorLocation(URRMathUtils::GetRandomSphericalPosition(
            InTargetLocation, CameraProperties.DistanceRangeInCm, CameraProperties.HeightRangeInCm));
    }
    SetActorRotation((InTargetLocation - GetActorLocation()).ToOrientationQuat());
}

void ARRCamera::RandomizeFoV()
{
    CameraComponent->FieldOfView = URRMathUtils::GetRandomFloatInRange(CameraProperties.HFoVRangeInDegree);
}

void ARRCamera::RandomizePose(const FVector& InBaseLocation, bool bIsRandomLocationOnly)
{
    const FVector randomLocation = URRMathUtils::GetRandomSphericalPosition(
        InBaseLocation, CameraProperties.DistanceRangeInCm, CameraProperties.HeightRangeInCm);

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
    return ActorCommon->SceneFloor
               ? FVector::Dist(CameraComponent->GetComponentLocation(), ActorCommon->SceneFloor->GetActorLocation())
               : 0.f;
}
