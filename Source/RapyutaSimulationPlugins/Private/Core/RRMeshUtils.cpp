// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRMeshUtils.h"
// Native
#include <exception>
#include <string>

// UE
#include "DrawDebugHelpers.h"
#include "HAL/FileManagerGeneric.h"
#include "Materials/MaterialInterface.h"
#include "Misc/FileHelper.h"
#include "ProceduralMeshComponent.h"

// Assimp
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "assimp/vector3.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRThreadUtils.h"
#include "RapyutaSimulationPlugins.h"

void URRMeshUtils::ProcessMeshNode(aiNode* InNode,
                                   const aiScene* InScene,
                                   int InParentNodeIndex,
                                   int* InCurrentIndex,
                                   FRRMeshData& OutMeshData)
{
    FRRMeshNode meshNode;
    meshNode.NodeParentIndex = InParentNodeIndex;

    aiMatrix4x4 nodeTransf = InNode->mTransformation;
    meshNode.RelativeTransform = FTransform(FMatrix(FPlane(nodeTransf.a1, nodeTransf.b1, nodeTransf.c1, nodeTransf.d1),
                                                    FPlane(nodeTransf.a2, nodeTransf.b2, nodeTransf.c2, nodeTransf.d2),
                                                    FPlane(nodeTransf.a3, nodeTransf.b3, nodeTransf.c3, nodeTransf.d3),
                                                    FPlane(nodeTransf.a4, nodeTransf.b4, nodeTransf.c4, nodeTransf.d4)));

#if RAPYUTA_MESH_UTILS_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Display,
                     TEXT("Node(%s): mNumMeshes: %d, mNumChildren: %d"),
                     *FString(InNode->mName.C_Str()),
                     InNode->mNumMeshes,
                     InNode->mNumChildren);
#endif
    if ((InNode->mNumMeshes > 0) && InNode->mMeshes)
    {
        for (auto i = 0; i < InNode->mNumMeshes; ++i)
        {
            auto meshIndex = InNode->mMeshes[i];
#if RAPYUTA_MESH_UTILS_DEBUG
            UE_LOG_WITH_INFO(LogRapyutaCore, Log, TEXT("Loading Mesh at index: %d"), meshIndex);
#endif
            aiMesh* mesh = InScene->mMeshes[meshIndex];
            if (mesh)
            {
                meshNode.Meshes.Emplace(ProcessMesh(mesh));
            }
        }

        // Only add [meshNode] having mesh data to [OutMeshData]'s Nodes
        if (meshNode.Meshes.Num() > 0)
        {
            OutMeshData.Nodes.Emplace(MoveTemp(meshNode));
        }
    }

    // Regardless of parent node having mesh or not
    // -> Keep processing child nodes
    int currentParentIndex = *InCurrentIndex;
    for (auto i = 0; i < InNode->mNumChildren; ++i)
    {
        (*InCurrentIndex)++;
        ProcessMeshNode(InNode->mChildren[i], InScene, currentParentIndex, InCurrentIndex, OutMeshData);
    }
}

FRRMeshNodeData URRMeshUtils::ProcessMesh(aiMesh* InMesh)
{
    FRRMeshNodeData outMeshNodeData;
    const bool bHasTangents = InMesh->HasTangentsAndBitangents();
    const bool bHasNormals = InMesh->HasNormals();
    const bool bHasFaces = InMesh->HasFaces();

#if RAPYUTA_MESH_UTILS_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Warning,
                     TEXT("ProcessMesh - Vertices num: %u Faces num: %u %d"),
                     InMesh->mNumVertices,
                     InMesh->mNumFaces,
                     bHasFaces);
#endif
    // Fetch mesh data, also Converting handedness from Assimp(right) ->UE (left)
    for (auto i = 0; i < InMesh->mNumVertices; ++i)
    {
        // [Vertices] --
        outMeshNodeData.Vertices.Emplace(
            URRConversionUtils::ConvertHandedness(FVector(InMesh->mVertices[i].x, InMesh->mVertices[i].y, InMesh->mVertices[i].z)));

        // [VertexColors] --
        const aiColor4D* colors = InMesh->mColors[0];
        outMeshNodeData.VertexColors.Emplace(colors ? FColor(colors[i].r, colors[i].g, colors[i].b, colors[i].a) : FColor::Black);

        // [Normals] --
        outMeshNodeData.Normals.Emplace(bHasNormals ? URRConversionUtils::ConvertHandedness(FVector(
                                                          InMesh->mNormals[i].x, InMesh->mNormals[i].y, InMesh->mNormals[i].z))
                                                    : FVector::ZeroVector);

        // [UVs] --
        // UVs have already been flipped with [aiProcess_FlipUVs] flag
        const aiVector3D* textureCoords = InMesh->mTextureCoords[0];
        outMeshNodeData.UVs.Emplace(
            textureCoords ? FVector2D(static_cast<double>(textureCoords[i].x), static_cast<double>(textureCoords[i].y))
                          : FVector2D::ZeroVector);
        outMeshNodeData.UV2fs.Emplace(FVector2f(outMeshNodeData.UVs.Last()));

        // [Tangents] --
        outMeshNodeData.ProcTangents.Emplace(
            bHasTangents ? FProcMeshTangent(InMesh->mTangents[i].x, -InMesh->mTangents[i].y, InMesh->mTangents[i].z)
                         : FProcMeshTangent());
    }

    // [BoneInfluences] --
    for (auto bi = 0; bi < InMesh->mNumBones; ++bi)
    {
        const auto& bone = InMesh->mBones[bi];
        if (bone)
        {
#if RAPYUTA_MESH_UTILS_DEBUG
            UE_LOG_WITH_INFO(
                LogRapyutaCore, Warning, TEXT("Bone %s mNumWeights: %u"), *FString(bone->mName.data), bone->mNumWeights);
#endif
            for (auto wi = 0; wi < bone->mNumWeights; ++wi)
            {
                const auto& boneWeight = bone->mWeights[wi];
                FRRBoneInfluence boneInfluence;
                boneInfluence.BoneIndex = bi;
                boneInfluence.Weight = boneWeight.mWeight;
                boneInfluence.VertexIndex = boneWeight.mVertexId;
                outMeshNodeData.BoneInfluences.Emplace(MoveTemp(boneInfluence));
            }
        }
    }

    // [Triangles/Faces' indices]
    if (bHasFaces)
    {
#if RAPYUTA_MESH_UTILS_DEBUG
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("mNumFaces: %u at %u"), InMesh->mNumFaces, InMesh->mFaces);
#endif
        for (auto f = 0; f < InMesh->mNumFaces; ++f)
        {
            const aiFace& face = InMesh->mFaces[f];
            if (nullptr != face.mIndices)
            {
#if RAPYUTA_SIM_DEBUG
                UE_LOG_WITH_INFO(
                    LogRapyutaCore, Warning, TEXT("face[%d].mNumIndices: %u at %u"), f, face.mNumIndices, face.mIndices);
#endif
                for (auto i = 0; i < face.mNumIndices; ++i)
                {
                    outMeshNodeData.TriangleIndices.Emplace(face.mIndices[i]);
                    // auto const &uv = InMesh->mTextureCoords[0][face.mIndices[i]];
                }
            }
        }
    }

    // [MaterialIndex]
    outMeshNodeData.MaterialIndex = InMesh->mMaterialIndex;

    return outMeshNodeData;
}

bool URRMeshUtils::ProcessTexture(aiMaterial* InMaterial,
                                  const aiTextureType InTextureType,
                                  const TCHAR* InTextureTypeName,
                                  const FString& InTextureBasePath,
                                  UMaterialInstanceDynamic* OutUEMaterial)
{
    static uint64 sTextureNameCount = 0;
#if RAPYUTA_SIM_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore,
                     Warning,
                     TEXT("Loading Material texture map [%s]...%d"),
                     InTextureTypeName,
                     InMaterial->GetTextureCount(InTextureType));
#endif

    aiString outTextureName;
    if (aiReturn_SUCCESS == InMaterial->GetTexture(InTextureType, 0, &outTextureName))
    {
        const FString fullTexturePath = FPaths::ConvertRelativePathToFull(InTextureBasePath, outTextureName.data);
        UTexture* ueTexture = URRCoreUtils::LoadImageToTexture(
            fullTexturePath, FString::Printf(TEXT("%d%s"), ++sTextureNameCount, InTextureTypeName));
        if (ueTexture != nullptr)
        {
            URRThreadUtils::DoTaskInGameThread([OutUEMaterial, InTextureTypeName, ueTexture]()
                                               { OutUEMaterial->SetTextureParameterValue(InTextureTypeName, ueTexture); });
            return true;
        }
        else
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Error,
                             TEXT("URRCoreUtils::LoadImageToTexture [%s] map failed %s"),
                             InTextureTypeName,
                             *fullTexturePath);
            return false;
        }
    }
    else
    {
        return false;
    }
}

void URRMeshUtils::ProcessMaterial(aiMaterial* InMaterial, const FString& InMeshFilePath, FRRMeshData& OutMeshData)
{
    static constexpr const TCHAR* MATERIAL_PARAM_NAME_BASE_COLOR = TEXT("BaseColor");
    static constexpr const TCHAR* MATERIAL_PARAM_NAME_NORMAL = TEXT("Normal");
    static constexpr const TCHAR* MATERIAL_PARAM_NAME_AMBIENT = TEXT("Ambient");
    static constexpr const TCHAR* MATERIAL_PARAM_NAME_SPECULAR = TEXT("Specular");
    static constexpr const TCHAR* MATERIAL_PARAM_NAME_EMISSIVE = TEXT("Emissive");
    static constexpr const TCHAR* MATERIAL_PARAM_NAME_METALLIC = TEXT("Metallic");
    static constexpr const TCHAR* MATERIAL_PARAM_NAME_ROUGHNESS = TEXT("Roughness");

    URRGameSingleton* gameSingleton = URRGameSingleton::Get();
    UMaterialInstanceDynamic* ueMaterial =
        UMaterialInstanceDynamic::Create(gameSingleton->GetMaterial(URRGameSingleton::MATERIAL_NAME_PROP_MASTER), gameSingleton);

#if RAPYUTA_SIM_DEBUG
    const FString fullMeshPath = FPaths::GetPath(InMeshFilePath);
    ProcessTexture(InMaterial, aiTextureType_BASE_COLOR, MATERIAL_PARAM_NAME_BASE_COLOR, fullMeshPath, ueMaterial);
    ProcessTexture(InMaterial, aiTextureType_NORMALS, MATERIAL_PARAM_NAME_NORMAL, fullMeshPath, ueMaterial);
    ProcessTexture(InMaterial, aiTextureType_AMBIENT, MATERIAL_PARAM_NAME_AMBIENT, fullMeshPath, ueMaterial);
    ProcessTexture(InMaterial, aiTextureType_SPECULAR, MATERIAL_PARAM_NAME_SPECULAR, fullMeshPath, ueMaterial);
    ProcessTexture(InMaterial, aiTextureType_EMISSION_COLOR, MATERIAL_PARAM_NAME_EMISSIVE, fullMeshPath, ueMaterial);
    ProcessTexture(InMaterial, aiTextureType_METALNESS, MATERIAL_PARAM_NAME_METALLIC, fullMeshPath, ueMaterial);
    ProcessTexture(InMaterial, aiTextureType_DIFFUSE_ROUGHNESS, MATERIAL_PARAM_NAME_ROUGHNESS, fullMeshPath, ueMaterial);
#endif

    auto fToLinearColor = [](const aiColor4D& InColor)
    {
        // https://stackoverflow.com/questions/12524623/what-are-the-practical-differences-when-working-with-colors-in-a-linear-vs-a-no
        // [aiColor4D] is already in [0, 1]
        return FLinearColor(InColor.r, InColor.g, InColor.b, InColor.a);
    };
    aiColor4D color;
    if (AI_SUCCESS == InMaterial->Get(AI_MATKEY_COLOR_DIFFUSE, color))
    {
        URRThreadUtils::DoTaskInGameThread([ueMaterial, linearColor = fToLinearColor(color)]()
                                           { ueMaterial->SetVectorParameterValue(MATERIAL_PARAM_NAME_BASE_COLOR, linearColor); });
    }
    if (AI_SUCCESS == InMaterial->Get(AI_MATKEY_COLOR_SPECULAR, color))
    {
        URRThreadUtils::DoTaskInGameThread([ueMaterial, linearColor = fToLinearColor(color)]()
                                           { ueMaterial->SetVectorParameterValue(MATERIAL_PARAM_NAME_SPECULAR, linearColor); });
    }
    if (AI_SUCCESS == InMaterial->Get(AI_MATKEY_COLOR_EMISSIVE, color))
    {
        URRThreadUtils::DoTaskInGameThread([ueMaterial, linearColor = fToLinearColor(color)]()
                                           { ueMaterial->SetVectorParameterValue(MATERIAL_PARAM_NAME_EMISSIVE, linearColor); });
    }
    if (AI_SUCCESS == InMaterial->Get(AI_MATKEY_COLOR_REFLECTIVE, color))
    {
        URRThreadUtils::DoTaskInGameThread(
            [ueMaterial, linearColor = fToLinearColor(color)]()
            { ueMaterial->SetVectorParameterValue(MATERIAL_PARAM_NAME_ROUGHNESS, FLinearColor::White - linearColor); });
    }
    if (AI_SUCCESS == InMaterial->Get(AI_MATKEY_COLOR_AMBIENT, color))
    {
        URRThreadUtils::DoTaskInGameThread([ueMaterial, linearColor = fToLinearColor(color)]()
                                           { ueMaterial->SetVectorParameterValue(MATERIAL_PARAM_NAME_AMBIENT, linearColor); });
    }

    // Add into [MaterialInstances]
    OutMeshData.MaterialInstances.Add(ueMaterial);
}

FRRMeshData URRMeshUtils::LoadMeshFromFile(const FString& InMeshFilePath, Assimp::Importer& InMeshImporter, float InMeshScale)
{
    FRRMeshData outMeshData;
    if (false == FPaths::FileExists(InMeshFilePath))
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Error, TEXT("Runtime Mesh Loader: InMeshFilePath does not exist at [%s]."), *InMeshFilePath);
        return outMeshData;
    }

    // [scene] must be a const ptr as required by Assimp
    const aiScene* scene = nullptr;
    try
    {
        // http://assimp.sourceforge.net/lib_html/data.html
        // +X points to the right, -Z points away from the viewer into the screen and +Y points upwards
        // Assimp(right-handed) vs UE(left-handed),
        // so [aiProcess_MakeLeftHanded] is supposed to be used here also, but not sure why it just distorts the output
        // vertices, thus they will be converted manually later.
        uint32 flags = aiProcess_TransformUVCoords | aiProcess_GenUVCoords | aiProcess_FlipUVs;

        // Assimp(m) -> UE(cm), scaled by x100, which necessitates [aiProcess_GlobalScale]
        flags |= aiProcess_GlobalScale;
        InMeshImporter.SetPropertyFloat(AI_CONFIG_GLOBAL_SCALE_FACTOR_KEY, 100.f * InMeshScale);
        if (InMeshFilePath.EndsWith(TEXT(".dae")))
        {
            InMeshImporter.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
        }

#if 0    // This is only required if the meshes are exported by Blender                                                           \
         // Rotate the mesh around X by -90                                                                                       \
         // InMeshImporter.SetPropertyBool(AI_CONFIG_PP_PTV_ADD_ROOT_TRANSFORMATION, true);                                       \
         // const aiMatrix4x4 initialMeshTransform(aiVector3D(0.f, 0.f, 0.f), aiQuaternion(0.f, 0.f, -90.f), aiVector3D(0.f, 0.f, \
         // 0.f));                                                                                                                \
         // InMeshImporter.SetPropertyMatrix(AI_CONFIG_PP_PTV_ROOT_TRANSFORMATION, initialMeshTransform);
#endif

        // Transform all meshes to World space, which necessitates [aiProcess_PreTransformVertices]
        flags |= aiProcess_PreTransformVertices;
        InMeshImporter.SetPropertyBool(AI_CONFIG_PP_PTV_KEEP_HIERARCHY, true);

        // https://github.com/assimp/assimp/issues/2093
        // http://wlosok.cz/procedural-mesh-in-ue4-1-triangle
        // assimp/code/Common/Importer.cpp#368
        // Note:
        // + [aiProcess_PreTransformVertices] cannot be used together with [aiProcess_OptimizeGraph],
        // same for [aiProcess_GenSmoothNormals] vs [aiProcess_GenNormals]
        // which is required for populating [scene->mRootNode]'s mesh data
        // + [aiProcess_MakeLeftHanded] This is supposed to be applied since Assimp mesh data is in right-handed,
        // but not clear yet why this makes mesh position incorrect?
        // + [aiProcess_FlipWindingOrder] makes CW while UE has CCW vertice winding order already, which makes a face' normal
        // face outward. aiProcessPreset_TargetRealtime_Fast | aiProcessPreset_TargetRealtime_Quality
        scene =
            InMeshImporter.ReadFile(URRCoreUtils::FToStdString(InMeshFilePath).c_str(),
                                    flags | (aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace | aiProcess_Triangulate |
                                             aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_FindInvalidData));
    }
    catch (std::exception&)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Exception: %s"), *InMeshFilePath);
        return outMeshData;
    }

    if (nullptr == scene)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Error: %s - %s"), *FString(InMeshImporter.GetErrorString()), *InMeshFilePath);
        return outMeshData;
    }
    else if (false == scene->HasMeshes())
    {
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Error, TEXT("Scene has no mesh: %s - %s"), *FString(InMeshImporter.GetErrorString()), *InMeshFilePath);
        return outMeshData;
    }
    else if (nullptr == scene->mRootNode)
    {
        // (Note) [scene->mRootNode->mNumMeshes] could be zero but its children should also have meshes
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Error, TEXT("NULL ROOT NODE: %s - %s"), *FString(InMeshImporter.GetErrorString()), *InMeshFilePath);
        return outMeshData;
    }

    float unitScaleFactor = 1.f;
    scene->mMetaData->Get("UnitScaleFactor", unitScaleFactor);
#if RAPYUTA_MESH_UTILS_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("UnitScaleFactor: %f %s"), unitScaleFactor, *InMeshFilePath);
    UE_LOG_WITH_INFO(
        LogRapyutaCore, Warning, TEXT("MESHES NUM: Scene(%u) RootNode(%u)"), scene->mNumMeshes, scene->mRootNode->mNumMeshes);
#endif

    // [Meshes] --
    int nodeIndex = 0;
    int* nodeIndexPtr = &nodeIndex;
    ProcessMeshNode(scene->mRootNode, scene, -1, nodeIndexPtr, outMeshData);

#if RAPYUTA_MESH_UTILS_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("MATERIALS NUM: %d"), scene->mNumMaterials);
#endif
    // Create Material instance dynamic in [outMeshData]
    // [Materials/Textures] --
    if (scene->HasMaterials())
    {
        for (auto i = 0; i < scene->mNumMaterials; ++i)
        {
            ProcessMaterial(scene->mMaterials[i], InMeshFilePath, outMeshData);
        }
    }

#if RAPYUTA_MESH_UTILS_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("NODES NUM: %d"), outMeshData.Nodes.Num());
#endif
    outMeshData.bIsValid = (outMeshData.Nodes.Num() > 0);
    return outMeshData;
}
