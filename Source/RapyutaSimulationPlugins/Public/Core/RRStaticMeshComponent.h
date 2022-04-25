// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Components/StaticMeshComponent.h"
#include "CoreMinimal.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMeshData.h"

#include "RRStaticMeshComponent.generated.h"

// [URRStaticMeshComponent] - COMPONENT OF [ARRMeshActor] --
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRStaticMeshComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
    friend class URRObjectUtils;

public:
    URRStaticMeshComponent();
    static constexpr int8 CUSTOM_DEPTH_STENCIL_VOID = 0;
    void Initialize(bool bInIsStationary, bool bInIsPhysicsEnabled);
    FOnMeshCreationDone OnMeshCreationDone;

    UPROPERTY()
    int8 SceneInstanceId = 0;

    UPROPERTY()
    URRActorCommon* ActorCommon = nullptr;

    UPROPERTY()
    FString MeshUniqueName;
    void SetMesh(UStaticMesh* InStaticMesh);
    void SetMeshSize(const FVector& InSize)
    {
        SetWorldScale3D(InSize / GetStaticMesh()->GetBoundingBox().GetSize());
    }

    UPROPERTY()
    FTransform OriginRelativeTransform;

    UPROPERTY()
    bool bIsStationary = false;

    UFUNCTION()
    FVector GetSize() const;

    UFUNCTION()
    FVector GetExtent() const;

    UFUNCTION()
    FVector GetScaledExtent() const;

    UFUNCTION()
    void LockSelf();

    UFUNCTION()
    void HideSelf(bool IsHidden);

    // Collision/Overlapping --
    UFUNCTION()
    void SetCollisionModeAvailable(bool IsOn, bool IsHitEventEnabled = false);
    UFUNCTION()
    void EnableOverlapping();

protected:
    virtual void BeginPlay() override;
};
