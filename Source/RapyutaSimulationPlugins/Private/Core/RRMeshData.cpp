// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRMeshData.h"

TMap<FString, TSharedPtr<FRRMeshData>> FRRMeshData::MeshDataStore;

void FRRBoneProperty::PrintSelf() const
{
    UE_LOG(LogRapyutaCore, Display, TEXT("Bone Name: %s"), *Name);
    UE_LOG(LogRapyutaCore, Display, TEXT("- Rel Transform (to owner component): %s"), *RelTransform.ToString());
    UE_LOG(LogRapyutaCore, Display, TEXT("- MeshScale3D: %s"), *MeshScale3D.ToString());
    UE_LOG(LogRapyutaCore, Display, TEXT("- Index: %d ParentIndex: %d"), Index, ParentIndex);
}

void FRRMeshData::PrintSelf() const
{
    for (auto i = 0; i < Nodes.Num(); ++i)
    {
        for (auto j = 0; j < Nodes[i].Meshes.Num(); ++j)
        {
            UE_LOG(LogRapyutaCore, Display, TEXT("Node[%d]-Mesh[%d]:"), i, j);
            Nodes[i].Meshes[j].PrintSelf();
        }
    }
}

void FRRMeshNodeData::PrintSelf() const
{
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("- Vertices num: %d\n"
                "- Triangles num: %d\n"
                "- Normals num: %d\n"
                "- UVs num: %d\n"
                "- ProcTangents num: %d\n"
                "- BoneInfluences num: %d\n"),
           Vertices.Num(),
           TriangleIndices.Num(),
           Normals.Num(),
           UVs.Num(),
           ProcTangents.Num(),
           BoneInfluences.Num());
}
