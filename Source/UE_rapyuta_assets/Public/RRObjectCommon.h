#pragma once

// UE
#include "UObject/Object.h"

// RapyutaSim
#include "UE_rapyuta_assets.h"
#include "RRObjectCommon.generated.h"

// Note: For avoiding cyclic inclusion, only UE built-in source header files could be included herein.

// SIM RESOURCE DATA --
//
UENUM(BlueprintType)
enum class ERRResourceDataType : uint8
{
    NONE,
    // UASSET --
    UE_STATIC_MESH,
    UE_MATERIAL,
    UE_TEXTURE,

    // URDF/SDF--
    URDF_SDF,

    TOTAL
};

// The atomic Sim resource
USTRUCT()
struct UE_RAPYUTA_ASSETS_API FRRResource
{
    GENERATED_BODY()
    FRRResource()
    {
    }
    FRRResource(const FString& InUniqueName, const FSoftObjectPath& InAssetPath, UObject* InAssetData)
        : UniqueName(InUniqueName), AssetPath(InAssetPath), AssetData(InAssetData)
    {
    }

    UPROPERTY()
    FString UniqueName;

    UPROPERTY()
    FSoftObjectPath AssetPath;
    FString GetAssetPath() const
    {
        return AssetPath.ToString();
    }

    UPROPERTY()
    UObject* AssetData = nullptr;
};

USTRUCT()
struct UE_RAPYUTA_ASSETS_API FRRResourceInfo
{
    GENERATED_BODY()
    FRRResourceInfo()
    {
    }
    FRRResourceInfo(const ERRResourceDataType InDataType) : DataType(InDataType)
    {
    }

    UPROPERTY()
    ERRResourceDataType DataType = ERRResourceDataType::NONE;
    UPROPERTY()
    int32 ToBeAsyncLoadedResourceNum = 0;
    UPROPERTY()
    bool HasBeenAllLoaded = false;

    UPROPERTY()
    TMap<FString, FRRResource> Data;

    void AddResource(const FString& InUniqueName, const FSoftObjectPath& InAssetPath, UObject* InAssetData)
    {
        Data.Add(InUniqueName, FRRResource(InUniqueName, InAssetPath, InAssetData));
    }

    void Finalize()
    {
        DataType = ERRResourceDataType::NONE;
        ToBeAsyncLoadedResourceNum = 0;
        HasBeenAllLoaded = false;
        Data.Empty();
    }
};
