// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRMathUtils.h"

FRandomStream URRMathUtils::RandomStream = FRandomStream();

FMatrix URRMathUtils::ComputeViewMatrixFromObjectToTarget(const FTransform& InObjectTransform, const FVector& InTargetLocation)
{
    // Reference: UnrealEngine/Engine/Source/Runtime/UMG/Public/Components/Viewport.h - ComputeOrbitMatrix()
    const FRotator rotation = InObjectTransform.Rotator();
    const FVector location = InObjectTransform.GetTranslation();
    FTransform rotationTransform = FTransform(-InTargetLocation) * FTransform(FRotator(0, rotation.Yaw, 0)) *
                                   FTransform(FRotator(0, 0, rotation.Pitch)) *
                                   FTransform(FVector(0, (location - InTargetLocation).Size(), 0));

    FMatrix orbitMatrix = rotationTransform.ToMatrixNoScale() * FInverseRotationMatrix(FRotator(0, 90.f, 0));
    return orbitMatrix.InverseFast();
}
