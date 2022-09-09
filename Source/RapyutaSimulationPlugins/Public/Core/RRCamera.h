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
#include "VectorTypes.h"
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
    FVector2D DistanceRangeInCm = FVector2D(100.0, 300.0);

    UPROPERTY()
    FVector2D HeightRangeInCm = FVector2D(5.0, 100.0);

    UPROPERTY()
    FVector2D HFoVRangeInDegree = FVector2D(80.0, 120.0);

    FORCEINLINE void PrintSelf() const
    {
        UE_LOG(LogRapyutaCore, Display, TEXT("CameraProperties:"));
        UE_LOG(LogRapyutaCore, Display, TEXT("- DistanceRangeInCm: X=%3.3f Y=%3.3f"), DistanceRangeInCm.X, DistanceRangeInCm.Y);
        verify(FMath::IsWithinInclusive(DistanceRangeInCm.X, 0.0, DistanceRangeInCm.Y));
        UE_LOG(LogRapyutaCore, Display, TEXT("- HeightRangeInCm: X=%3.3f Y=%3.3f"), HeightRangeInCm.X, HeightRangeInCm.Y);
        verify(FMath::IsWithinInclusive(HeightRangeInCm.X, 0.0, HeightRangeInCm.Y));
        UE_LOG(LogRapyutaCore, Display, TEXT("- HFoVRangeInDegree: X=%3.3f Y=%3.3f"), HFoVRangeInDegree.X, HFoVRangeInDegree.Y);
        verify(FMath::IsWithinInclusive(HFoVRangeInDegree.X, 0.0, HFoVRangeInDegree.Y));
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
