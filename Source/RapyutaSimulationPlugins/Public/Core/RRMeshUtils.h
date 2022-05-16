/**
 * @file RRMeshUtils.h
 * @brief MeshUtils
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */


#pragma once

// UE
#include "Materials/MaterialInstanceDynamic.h"

// Assimp
#include "assimp/Importer.hpp"
#include "assimp/mesh.h"
#include "assimp/scene.h"

// UE
#include "ProceduralMeshComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRMeshData.h"

#include "RRMeshUtils.generated.h"

/**
 * @brief MeshUtils
 * @todo add documentation
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRMeshUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    static FRRMeshNodeData ProcessMesh(aiMesh* InMesh);
    // (Note) Use [int] instead of [int32 ,int64] due to the compatibility with Assimp's api
    static void ProcessMeshNode(aiNode* InNode,
                                const aiScene* InScene,
                                int InParentNodeIndex,
                                int* InCurrentIndex,
                                FRRMeshData& OutMeshData);

    static bool ProcessTexture(aiMaterial* InMaterial,
                               const aiTextureType InTextureType,
                               const TCHAR* InTextureTypeName,
                               const FString& InTextureBasePath,
                               UMaterialInstanceDynamic* OutUEMaterial);
    static void ProcessMaterial(aiMaterial* InMaterial, const FString& InMeshFilePath, FRRMeshData& OutMeshData);

    static FRRMeshData LoadMeshFromFile(const FString& InMeshFilePath, Assimp::Importer& InMeshImporter, float InMeshScale = 1.f);
};
