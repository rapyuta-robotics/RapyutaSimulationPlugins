/**
 * @file RRMeshData.h
 * @brief MeshData.
 * @sa  https://github.com/Chrizey91/RuntimeMeshLoader/blob/main/Source/RuntimeMeshLoader/Public/MeshLoader.h
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// Native
#include <string>

// UE
#include "Engine/Texture.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"

// Assimp
#include "assimp/Importer.hpp"
#include "assimp/mesh.h"
#include "assimp/scene.h"

// RapyutaSimulationPlugins
#include "RapyutaSimulationPlugins.h"

#include "RRMeshData.generated.h"

DECLARE_DELEGATE_TwoParams(FOnMeshCreationDone, bool /* bCreationResult */, UObject* /*MeshBodyComponent*/);

#define RAPYUTA_MESH_VISUAL_DEBUG (1)

/**
 * @brief todo
 * 
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRBoneProperty
{
    GENERATED_BODY()
public:
    UPROPERTY(VisibleAnywhere)
    FString Name;

    UPROPERTY(VisibleAnywhere)
    int32 Index = 0;

    UPROPERTY(VisibleAnywhere)
    int32 ParentIndex = 0;

    // Relative transform to the owner skinned mesh comp, as in EBoneSpaces::ComponentSpace
    UPROPERTY(VisibleAnywhere)
    FTransform RelTransform = FTransform::Identity;

    UPROPERTY(VisibleAnywhere)
    FVector MeshScale3D = FVector::OneVector;

    void PrintSelf() const;
};

/**
 * @brief todo
 * 
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRBoneInfluence
{
    GENERATED_BODY()

    UPROPERTY()
    float Weight = 0.f;
    UPROPERTY()
    int32 VertexIndex = 0;
    UPROPERTY()
    int32 BoneIndex = 0;
};

/**
 *	@brief Contains the vertices that are most dominated by that bone. Vertices are in Bone space.
 *	Not used at runtime, but useful for fitting physics assets etc.
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRBoneVertInfo
{
    GENERATED_BODY()

    // These must be of same length!
    UPROPERTY()
    TArray<FVector> Positions;
    UPROPERTY()
    TArray<FVector> Normals;
};

/**
 * @brief todo
 * 
 */
USTRUCT()
/**
 * @brief todo
 * 
 */
struct RAPYUTASIMULATIONPLUGINS_API FRRMeshNodeData
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FVector> Vertices;
    UPROPERTY()
    TArray<FColor> VertexColors;

    UPROPERTY()
    TArray<int32> TriangleIndices;

    UPROPERTY()
    TArray<FVector> Normals;

    UPROPERTY()
    TArray<FVector2D> UVs;

    UPROPERTY()
    TArray<FProcMeshTangent> ProcTangents;

    UPROPERTY()
    TArray<FRRBoneInfluence> BoneInfluences;

    UPROPERTY()
    uint32 MaterialIndex = 0;

    void Reset(uint64 InNum = 0)
    {
        Vertices.SetNumZeroed(InNum);
        TriangleIndices.SetNumZeroed(InNum);
        ProcTangents.SetNumZeroed(InNum);
    }

    void PrintSelf() const;
};

/**
 * @brief todo
 * 
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRMeshNode
{
    GENERATED_BODY()

    UPROPERTY()
    FTransform RelativeTransform = FTransform::Identity;

    UPROPERTY()
    int NodeParentIndex = 0;

    UPROPERTY()
    TArray<FRRMeshNodeData> Meshes;
};

/**
 * @brief todo
 * 
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRMeshData
{
    GENERATED_BODY()
private:
    static TMap<FString /* MeshUniqueName */, TSharedPtr<FRRMeshData>> MeshDataStore;

public:
    static void AddMeshData(const FString& InMeshUniqueName, const TSharedPtr<FRRMeshData>& InMeshData)
    {
        MeshDataStore.Add(InMeshUniqueName, InMeshData);
    }
    static TSharedPtr<FRRMeshData> GetMeshData(const FString& InMeshUniqueName)
    {
        return MeshDataStore.FindRef(InMeshUniqueName);
    }
    static bool IsMeshDataAvailable(const FString& InMeshUniqueName)
    {
        return MeshDataStore.Contains(InMeshUniqueName);
    }

    UPROPERTY()
    FString MeshUniqueName;

    // As doing async mesh loading, we need separate [MeshImporter] for 3D model files
    TSharedPtr<Assimp::Importer> MeshImporter = nullptr;

    UPROPERTY()
    bool bIsValid = false;
    bool IsValid() const
    {
        return bIsValid;
    }

    UPROPERTY()
    TArray<FRRMeshNode> Nodes;

    UPROPERTY()
    TArray<UMaterialInstanceDynamic*> MaterialInstances;

    void Reset()
    {
        Nodes.Reset();
        MaterialInstances.Reset();
    }

    void PrintSelf() const;

    void TransformBy(const FTransform& InTransform)
    {
        for (auto& meshNode : Nodes)
        {
            for (auto& mesh : meshNode.Meshes)
            {
                for (auto& vertex : mesh.Vertices)
                {
                    vertex = InTransform.TransformPosition(vertex);
                }

                for (auto& normal : mesh.Normals)
                {
                    normal = InTransform.TransformVectorNoScale(normal);
                }

                for (auto& procTangent : mesh.ProcTangents)
                {
                    procTangent.TangentX = InTransform.TransformVectorNoScale(procTangent.TangentX);
                }
            }
        }
    }

    int32 GetVerticesNum() const
    {
        int32 verticesNum = 0;
        for (const auto& meshNode : Nodes)
        {
            for (const auto& mesh : meshNode.Meshes)
            {
                verticesNum += mesh.Vertices.Num();
            }
        }
        return verticesNum;
    }

    bool HasVertexColors() const
    {
        for (const auto& meshNode : Nodes)
        {
            for (const auto& mesh : meshNode.Meshes)
            {
                if (mesh.VertexColors.Num() > 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

    int32 GetIndicesNum() const
    {
        int32 indicesNum = 0;
        for (const auto& meshNode : Nodes)
        {
            for (const auto& mesh : meshNode.Meshes)
            {
                indicesNum += mesh.TriangleIndices.Num();
            }
        }
        return indicesNum;
    }

    int32 GetUVsNum() const
    {
        for (const auto& meshNode : Nodes)
        {
            for (const auto& mesh : meshNode.Meshes)
            {
                return mesh.UVs.Num();
            }
        }
        return 0;
    }
};
