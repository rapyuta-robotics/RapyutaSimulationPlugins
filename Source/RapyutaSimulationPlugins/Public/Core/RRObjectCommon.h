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
 * @brief File types
 */
UENUM()
enum class ERRFileType : uint8
{
    NONE,
    UASSET,    // UE Asset file
    PAK,
    INI,
    YAML,
    ZIP,

    // Image
    IMAGE_JPG,
    IMAGE_GRAYSCALE_JPG,
    IMAGE_PNG,
    IMAGE_TGA,
    IMAGE_EXR,
    IMAGE_HDR,

    // Light Profile
    LIGHT_PROFILE_IES,

    // Meta Data
    JSON,

    // 3D Description Format
    URDF,
    SDF,
    GAZEBO_WORLD,
    MJCF,    // MuJoCo

    // 3D CAD
    CAD_FBX,
    CAD_OBJ,
    CAD_STL,
    CAD_DAE,
    TOTAL
};

/**
 * @brief Shape types
 */
UENUM()
enum class ERRShapeType : uint8
{
    NONE,
    PLANE,
    BOX,
    CYLINDER,
    SPHERE,
    CAPSULE,
    MESH,
    TOTAL
};

/**
 * @brief Sim resource(Uassets) data types
 *
 */
UENUM(BlueprintType)
enum class ERRResourceDataType : uint8
{
    NONE,
    // UASSET --
    UE_PAK,
    UE_STATIC_MESH,
    UE_SKELETAL_MESH,
    UE_SKELETON,
    UE_PHYSICS_ASSET,
    UE_MATERIAL,
    UE_PHYSICAL_MATERIAL,
    UE_TEXTURE,
    UE_DATA_TABLE,

    // UOBJECT --
    // Cooked collision data
    UE_BODY_SETUP,

    TOTAL
};

/**
 * @brief The atomic Sim resouces(Uassets)
 * #FRRResourceInfo has TMap of this to store pointers to asset of each #ERRResourceDataType
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
 * @brief Structure to store resources(Uassets) information.
 * #RRGameSingleton has TMap of this to store info for each #ERRResourceDataType
 * #Data is TMap of FRRResource which has pointer to AssetData.
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
    bool bHasBeenAllLoaded = false;

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
        bHasBeenAllLoaded = false;

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
                if (resource.AssetData)
                {
                    resource.AssetData->MarkAsGarbage();
                }
            }
        }
        Data.Reset();
    }
};

USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRMaterialProperty
{
    GENERATED_BODY()

    // These property names are defined in master material by the artist
    static constexpr const TCHAR* PROP_NAME_ALBEDO = TEXT("AlbedoTexture");
    static constexpr const TCHAR* PROP_NAME_ORM = TEXT("MergeMapInput");
    static constexpr const TCHAR* PROP_NAME_NORMAL = TEXT("MainNormalInput");
    static constexpr const TCHAR* PROP_NAME_MASK = TEXT("MaskSelection");
    static constexpr const TCHAR* PROP_NAME_COLOR_ALBEDO = TEXT("ColorAlbedo");
    static constexpr const TCHAR* PROP_NAME_EMISSIVE_STRENGTH = TEXT("EmissiveStrength");

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
        UE_LOG(LogTemp, Warning, TEXT("Material: %s"), *Name);
        UE_LOG(LogTemp, Display, TEXT("- Color: %s"), *Color.ToString());
        UE_LOG(LogTemp, Display, TEXT("- AlbedoTextureNameList: %s"), *FString::Join(AlbedoTextureNameList, TEXT(",")));
        UE_LOG(LogTemp,
               Display,
               TEXT("- AlbedoColorList: %s"),
               *FString::JoinBy(AlbedoColorList, TEXT(","), [](const FLinearColor& InColor) { return InColor.ToString(); }));
        UE_LOG(LogTemp, Display, TEXT("- MaskTextureName: %s"), *MaskTextureName);
        UE_LOG(LogTemp, Display, TEXT("- ORMTextureName: %s"), *ORMTextureName);
        UE_LOG(LogTemp, Display, TEXT("- NormalTextureName: %s"), *NormalTextureName);
    }

    bool IsValid() const
    {
        return !Name.IsEmpty();
    }

    bool HasTexture(const FString& InTextureName) const
    {
        return AlbedoTextureNameList.Contains(InTextureName) || (InTextureName == MaskTextureName) ||
               (InTextureName == ORMTextureName) || (InTextureName == NormalTextureName);
    }
};
