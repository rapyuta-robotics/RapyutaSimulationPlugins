/**
 * @file RRObjectCommon.h
 * @brief Common objects.
 * @todo add documentation
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "PhysicsEngine/BodySetup.h"
#include "UObject/Object.h"

// RapyutaSim
#include "RapyutaSimulationPlugins.h"

#include "RRObjectCommon.generated.h"

// Note: For avoiding cyclic inclusion, only UE built-in source header files could be included herein.

/**
 * @brief Sim resource data
 *
 */
UENUM(BlueprintType)
enum class ERRResourceDataType : uint8
{
    NONE,
    // UASSET --
    UE_STATIC_MESH,
    UE_SKELETAL_MESH,
    UE_SKELETON,
    UE_PHYSICS_ASSET,
    UE_MATERIAL,
    UE_TEXTURE,

    // UOBJECT --
    // Cooked collision data
    UE_BODY_SETUP,

    TOTAL
};

/**
 * @brief The atomic Sim resource
 *
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRResource
{
    GENERATED_BODY()

    /**
     * @brief Construct a new FRRResource object
     *
     */
    FRRResource()
    {
    }

    /**
     * @brief Construct a new FRRResource object
     *
     * @param InUniqueName
     * @param InAssetPath
     * @param InAssetData
     */
    FRRResource(const FString& InUniqueName, const FSoftObjectPath& InAssetPath, UObject* InAssetData)
        : UniqueName(InUniqueName), AssetPath(InAssetPath), AssetData(InAssetData)
    {
    }

    UPROPERTY()
    FString UniqueName;

    UPROPERTY()
    FSoftObjectPath AssetPath;

    /**
     * @brief Get the Asset Path.
     *
     * @return FString
     */
    FString GetAssetPath() const
    {
        return AssetPath.ToString();
    }

    UPROPERTY()
    UObject* AssetData = nullptr;
};

/**
 * @brief Structure to store resources information.
 *
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRResourceInfo
{
    GENERATED_BODY()

    /**
     * @brief Construct a new FRRResourceInfo
     *
     */
    FRRResourceInfo()
    {
    }

    /**
     * @brief Construct a new FRRResourceInfo
     *
     * @param InDataType
     */
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

    /**
     * @brief Finalize object by using UBodySetup::ClearPhysicsMeshes and MarkAsGarbage.
     * BodySetup's collision mesh data are manually created from the underlying Physics engine, thus needs manual flush.
     * Besides, body setup data is shared among proc mesh comps, thus could not be destroyed in a dtor or OnComponentDestroyed()
     * @sa [UBodySetup::ClearPhysicsMeshes](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/PhysicsEngine/UBodySetup/ClearPhysicsMeshes/)
     * @sa [MarkAsGarbage](https://docs.unrealengine.com/5.0/en-US/API/Runtime/Engine/EdGraph/UEdGraphPin/MarkAsGarbage/)
     */
    void Finalize()
    {
        DataType = ERRResourceDataType::NONE;
        ToBeAsyncLoadedResourceNum = 0;
        HasBeenAllLoaded = false;

        // BodySetup's collision mesh data are manually created from the underlying Physics engine,
        // thus needs manual flush
        // Besides, body setup data is shared among proc mesh comps, thus could not be destroyed in a dtor or
        // OnComponentDestroyed()
        if (ERRResourceDataType::UE_BODY_SETUP == DataType)
        {
            for (auto& data : Data)
            {
                UBodySetup* bodySetup = Cast<UBodySetup>(data.Value.AssetData);
                if (IsValid(bodySetup))
                {
                    bodySetup->ClearPhysicsMeshes();
                }
            }
        }
        else
        {
            for (auto& [_, resource] : Data)
            {
                resource.AssetData->MarkAsGarbage();
            }
        }
        Data.Reset();
    }
};

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRMaterialProperty
{
    // These property names are defined in master material by the artist
    static constexpr const TCHAR* PROP_NAME_ALBEDO = TEXT("AlbedoTexture");
    static constexpr const TCHAR* PROP_NAME_ORM = TEXT("MergeMapInput");
    static constexpr const TCHAR* PROP_NAME_NORMAL = TEXT("MainNormalInput");
    static constexpr const TCHAR* PROP_NAME_MASK = TEXT("MaskSelection");
    static constexpr const TCHAR* PROP_NAME_COLOR_ALBEDO = TEXT("ColorAlbedo");

    GENERATED_BODY()
    UPROPERTY(VisibleAnywhere)
    FString Name;
    UPROPERTY(VisibleAnywhere)
    FLinearColor Color = FLinearColor::Transparent;
    UPROPERTY(VisibleAnywhere)
    TArray<FString> AlbedoTextureNameList;
    UPROPERTY(VisibleAnywhere)
    TArray<FLinearColor> AlbedoColorList;
    UPROPERTY(VisibleAnywhere)
    FString MaskTextureName;
    UPROPERTY(VisibleAnywhere)
    FString ORMTextureName;
    UPROPERTY(VisibleAnywhere)
    FString NormalTextureName;

    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Material: %s"), *Name);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Color: %s"), *Color.ToString());
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- AlbedoTextureNameList: %s"), *FString::Join(AlbedoTextureNameList, TEXT(",")));
        UE_LOG_WITH_INFO(
            LogTemp,
            Display,
            TEXT("- AlbedoColorList: %s"),
            *FString::JoinBy(AlbedoColorList, TEXT(","), [](const FLinearColor& InColor) { return InColor.ToString(); }));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- MaskTextureName: %s"), *MaskTextureName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- ORMTextureName: %s"), *ORMTextureName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- NormalTextureName: %s"), *NormalTextureName);
    }

    bool HasTexture(const FString& InTextureName)
    {
        return AlbedoTextureNameList.Contains(InTextureName) || (InTextureName == MaskTextureName) ||
               (InTextureName == ORMTextureName) || (InTextureName == NormalTextureName);
    }
};
