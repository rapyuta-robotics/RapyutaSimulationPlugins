/**
 * @file RRCamera.h
 * @brief Standalone camera actor formed by #UCameraComponent
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Camera/CameraComponent.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Core/RRMathUtils.h"
#include "Core/RRUObjectUtils.h"

#include "RRCamera.generated.h"

/**
 * @brief Camera user properties
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRCameraProperties
{
    GENERATED_BODY()

    UPROPERTY()
    FVector2f DistanceRangeInCm = FVector2f(100.f, 300.f);

    UPROPERTY()
    FVector2f HeightRangeInCm = FVector2f(5.f, 100.f);

    UPROPERTY()
    FVector2f HFoVRangeInDegree = FVector2f(80.f, 120.f);

    FORCEINLINE void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("CameraProperties:"));
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("- DistanceRangeInCm: %s"), *DistanceRangeInCm.ToString());
        verify(FMath::IsWithinInclusive(double(DistanceRangeInCm.X), double(0), double(DistanceRangeInCm.Y)));
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("- HeightRangeInCm: %s"), *HeightRangeInCm.ToString());
        verify(FMath::IsWithinInclusive(double(HeightRangeInCm.X), double(0), double(HeightRangeInCm.Y)));
        UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("- HFoVRangeInDegree: %s"), *HFoVRangeInDegree.ToString());
        verify(FMath::IsWithinInclusive(double(HFoVRangeInDegree.X), double(0), double(HFoVRangeInDegree.Y)));
    }
};

/**
 * @brief Standalone camera actor which can be placed in the level with #UCameraComponent.
 */
UCLASS(config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API ARRCamera : public ARRBaseActor
{
    GENERATED_BODY()

public:
    ARRCamera();

    UPROPERTY()
    UCameraComponent* CameraComponent = nullptr;

    UPROPERTY(config)
    FRRCameraProperties CameraProperties;

    virtual bool Initialize() override;
    void LookAt(const FVector& InTargetLocation,
                const FVector& InTargetNormal,
                const FVector2f& InDistanceRange,
                const FVector2f& InHeightRange,
                const FVector2f& InAngleRange,
                bool bMoveToNearTarget)
    {
        if (bMoveToNearTarget)
        {
            // In [Target]'s coordinate, rotate XAxis around its ZAxis a random angle in [InAngleRange]
            FVector randomDelta =
                FQuat(FVector::ZAxisVector, URRMathUtils::GetRandomFloatInRange(InAngleRange)).RotateVector(FVector::XAxisVector);

            // Determinate [randomDelta]'s corresponding self of [Target]'s XAxis
            randomDelta = FQuat::FindBetweenNormals(FVector::XAxisVector, InTargetNormal.GetSafeNormal()).RotateVector(randomDelta);

            randomDelta *= URRMathUtils::GetRandomFloatInRange(InDistanceRange);
            randomDelta.Z = URRMathUtils::GetRandomFloatInRange(InHeightRange);
            SetActorLocation(InTargetLocation + randomDelta);
        }
        SetActorRotation((InTargetLocation - GetActorLocation()).ToOrientationQuat());
    }
    void LookAt(const FVector& InTargetLocation, bool bMoveToNearTarget);
    template<typename T>
    void LookAt(const TArray<T*>& InTargetActors, bool bMoveToNearTarget)
    {
        LookAt(URRUObjectUtils::GetActorGroupCenter(InTargetActors), bMoveToNearTarget);
    }
    template<typename T>
    void LookAt(const TArray<TArray<T*>>& InTargetActorGroups, bool bMoveToNearTarget)
    {
        LookAt(URRUObjectUtils::GetActorGroupListCenter(InTargetActorGroups), bMoveToNearTarget);
    }
    void RandomizeFoV();
    void RandomizePose(const FVector& InBaseLocation, bool bIsRandomLocationOnly);

    float GetDistanceToFloor() const;
    template<typename T>
    float GetDistanceToActorsGroup(const TArray<T*>& InActors) const
    {
        return FVector::Dist(CameraComponent->GetComponentLocation(), URRUObjectUtils::GetActorGroupCenter(InActors));
    }

    template<typename T>
    float GetDistanceToActorGroupList(const TArray<TArray<T*>>& InActorGroups) const
    {
        return FVector::Dist(CameraComponent->GetComponentLocation(), URRUObjectUtils::GetActorGroupListCenter(InActorGroups));
    }

    float GetFocalLength() const
    {
        // HFOV = 2 * arctan( width / 2f )
        float halfFOV = FMath::DegreesToRadians(0.5f * CameraComponent->FieldOfView);
        return (0.5f * CameraComponent->OrthoWidth) / FMath::Tan(halfFOV);
    }

protected:
    virtual void PrintSimConfig() const override;
};
