// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRSkeletalMeshWrapper.h"

// UE
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "SkeletalRenderPublic.h"

// RapyutaSimRobotImporter
#include "Core/RRMeshData.h"

void RRSkeletalMeshWrapper::Init(USkeletalMeshComponent* InSkeletalMeshComponent)
{
    SkeletalMeshComponent = InSkeletalMeshComponent;
}

void RRSkeletalMeshWrapper::PrintInfo() const
{
    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[%s] SkeletalBody's PhysicsStateCreated %d - MeshObject %d %d %d"),
           *(SkeletalMeshComponent->GetName()),
           SkeletalMeshComponent->IsPhysicsStateCreated(),
           SkeletalMeshComponent->MeshObject->HaveValidDynamicData(),
           SkeletalMeshComponent->MeshObject->IsCPUSkinned(),
           SkeletalMeshComponent->MeshObject->GetLOD());

    UE_LOG(LogRapyutaCore,
           Warning,
           TEXT("[%s] SkeletalBody: Bodies %d - Constraints %d"),
           *(SkeletalMeshComponent->GetName()),
           SkeletalMeshComponent->Bodies.Num(),
           SkeletalMeshComponent->Constraints.Num());

    // Bodies
    for (int32 bodyIdx = 0; bodyIdx < SkeletalMeshComponent->Bodies.Num(); ++bodyIdx)
    {
        FBodyInstance* body = SkeletalMeshComponent->Bodies[bodyIdx];
        if (body)
        {
            if (body->BodySetup.IsValid())
            {
                UE_LOG(LogRapyutaCore,
                       Warning,
                       TEXT("- Body[%s] BodyIndex %d BoneIndex %d BoneMass %f"),
                       *body->BodySetup->BoneName.ToString(),
                       body->InstanceBodyIndex,
                       body->InstanceBoneIndex,
                       SkeletalMeshComponent->GetBoneMass(body->BodySetup->BoneName));
            }
            else
            {
                UE_LOG(LogRapyutaCore,
                       Warning,
                       TEXT("- Bodies[%d]'s BodySetup is null, probably destroyed due to no collision"),
                       bodyIdx);
            }
        }
        else
        {
            UE_LOG(LogRapyutaCore, Error, TEXT("- Bodies[%d] is null"), bodyIdx);
        }
    }

    // SkeletalBodySetups
    for (const auto& skBodySetup : SkeletalMeshComponent->GetPhysicsAsset()->SkeletalBodySetups)
    {
        if (skBodySetup)
        {
            UE_LOG(LogRapyutaCore,
                   Warning,
                   TEXT("- [%s]'s DefaultBodyInstance: BodyIndex %d BoneIndex %d MassOverride %f"),
                   *skBodySetup->GetName(),
                   skBodySetup->DefaultInstance.InstanceBodyIndex,
                   skBodySetup->DefaultInstance.InstanceBoneIndex,
                   skBodySetup->DefaultInstance.GetMassOverride());
        }
    }

    // Constraints
    for (const auto& ci : SkeletalMeshComponent->Constraints)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("%u Constraint %s"), ci, *ci->JointName.ToString());
    }
    for (const auto& t : SkeletalMeshComponent->GetPhysicsAsset()->ConstraintSetup)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("%u ConstraintSetup %s"), t, *t->DefaultInstance.JointName.ToString());
    }

#if WITH_EDITOR
    for (const auto& profileName : SkeletalMeshComponent->GetPhysicsAsset()->GetConstraintProfileNames())
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("Constraint profile %s"), *profileName.ToString());
    }
#endif

    // PhysicsAsset
    UE_LOG(LogRapyutaCore, Warning, TEXT("PhysicsAsset: %s"), *SkeletalMeshComponent->GetPhysicsAsset()->GetDesc());
    for (const auto bodySetup : SkeletalMeshComponent->GetPhysicsAsset()->BodySetupIndexMap)
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("- Body %d %s"), bodySetup.Value, *bodySetup.Key.ToString());
    }
}
