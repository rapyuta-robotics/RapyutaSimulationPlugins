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
        UE_LOG(LogRapyutaCore, Display, TEXT("CameraProperties:"));
        UE_LOG(LogRapyutaCore, Display, TEXT("- DistanceRangeInCm: %s"), *DistanceRangeInCm.ToString());
        verify(FMath::IsWithinInclusive(DistanceRangeInCm.X, 0.f, DistanceRangeInCm.Y));
        UE_LOG(LogRapyutaCore, Display, TEXT("- HeightRangeInCm: %s"), *HeightRangeInCm.ToString());
        verify(FMath::IsWithinInclusive(HeightRangeInCm.X, 0.f, HeightRangeInCm.Y));
        UE_LOG(LogRapyutaCore, Display, TEXT("- HFoVRangeInDegree: %s"), *HFoVRangeInDegree.ToString());
        verify(FMath::IsWithinInclusive(HFoVRangeInDegree.X, 0.f, HFoVRangeInDegree.Y));
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
    void LookAt(const FVector& InTargetLocation);
    template<typename T>
    void LookAt(const TArray<T*>& InTargetActors)
    {
        LookAt(URRUObjectUtils::GetActorsGroupCenter(InTargetActors));
    }
    void RandomizeFoV();
    void RandomizePose(bool bIsRandomLocationOnly);
    float GetDistanceToFloor() const;
    float GetFocalLength() const
    {
        // HFOV = 2 * arctan( width / 2f )
        float halfFOV = FMath::DegreesToRadians(0.5f * CameraComponent->FieldOfView);
        return (0.5f * CameraComponent->OrthoWidth) / FMath::Tan(halfFOV);
    }

protected:
    virtual void PrintSimConfig() const override;
};
