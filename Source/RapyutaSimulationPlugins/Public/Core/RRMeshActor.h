// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRStaticMeshComponent.h"
#include "RapyutaSimulationPlugins.h"

#include "RRMeshActor.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRMeshActor : public ARRBaseActor
{
    GENERATED_BODY()
public:
    ARRMeshActor();

public:
    template<typename TActorSpawnInfo>
    bool InitializeWithSpawnInfo(const TActorSpawnInfo& InActorInfo)
    {
        ActorInfo = MakeShared<TActorSpawnInfo>(InActorInfo);

        // ACTOR INTIALIZING GENERAL INFO (Unique name, mesh list, material list, etc.)
        return Initialize();
    }

    virtual bool Initialize() override;
    virtual bool HasInitialized(bool bIsLogged = false) const override;
    virtual void Reset() override;

public:
    UPROPERTY(VisibleAnywhere)
    TArray<URRStaticMeshComponent*> MeshCompList;

    UPROPERTY(VisibleAnywhere)
    URRStaticMeshComponent* BaseMeshComp = nullptr;

public:
    void DrawTransform();

    URRStaticMeshComponent* GetMeshComponent(int32 Index = 0) const;
    TArray<URRStaticMeshComponent*> CreateMeshComponentList(USceneComponent* InParentComp,
                                                            const TArray<FString>& InMeshUniqueNameList,
                                                            const TArray<FTransform>& InMeshRelTransf = TArray<FTransform>());
    void SetCustomDepthEnabled(bool bIsCustomDepthEnabled);
    bool IsCustomDepthEnabled() const;
};
