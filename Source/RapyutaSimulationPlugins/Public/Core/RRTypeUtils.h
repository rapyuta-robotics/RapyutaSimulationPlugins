/**
 * @file RRTypeUtils.h
 * @brief UE type related utils
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "AssetRegistry/AssetRegistryModule.h"
#include "Kismet/BlueprintFunctionLibrary.h"

// RapyutaSimulationPlugins
#include "Core/RRObjectCommon.h"
#include "RapyutaSimulationPlugins.h"

#include "RRTypeUtils.generated.h"

/**
 * @brief UE/std type related utils
 *
 */
template<typename T>
struct TIsBoolean
{
    enum
    {
        Value = false
    };
};

template<>
struct TIsBoolean<bool>
{
    enum
    {
        Value = true
    };
};

template<typename T>
struct TIsCharPointer
{
    enum
    {
        Value = TAnd<TIsPointer<T>, TIsCharType<std::remove_cv_t<typename TRemovePointer<T>::Type>>>::Value
    };
};

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRTypeUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    // UE-TYPE RELATED UTILS ==
    //
    template<typename TEnum>
    FORCEINLINE static FString GetEnumValueAsString(const FString& InTypeName,
                                                    TEnum InEnumValue,
                                                    const TCHAR* InModuleName = nullptr)
    {
        const UEnum* EnumPtr = FindObject<UEnum>(
            nullptr,
            *FString::Printf(
                TEXT("/Script/%s.%s"), InModuleName ? InModuleName : RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME, *InTypeName),
            true);
        if (!EnumPtr)
        {
            return "Invalid";
        }

        return EnumPtr->GetDisplayNameTextByIndex(static_cast<int32>(InEnumValue)).ToString();    // Or GetNameByValue
    }

    template<typename TEnum>
    FORCEINLINE static FString GetEnumNameByValue(const FString& InTypeName, TEnum InEnumValue, const TCHAR* InModuleName = nullptr)
    {
        const UEnum* EnumPtr = FindObject<UEnum>(
            nullptr,
            *FString::Printf(
                TEXT("/Script/%s.%s"), InModuleName ? InModuleName : RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME, *InTypeName),
            true);
        if (!EnumPtr)
        {
            return "Invalid";
        }

        return EnumPtr->GetNameByValue(static_cast<int32>(InEnumValue)).ToString();    // Or GetNameByValue
    }

    FORCEINLINE static int8 GetEnumValueFromString(const FString& InTypeName,
                                                   const FString& InEnumStringValue,
                                                   const TCHAR* InModuleName = nullptr)
    {
        UEnum* EnumPtr = FindObject<UEnum>(
            nullptr,
            *FString::Printf(
                TEXT("/Script/%s.%s"), InModuleName ? InModuleName : RAPYUTA_SIMULATION_PLUGINS_MODULE_NAME, *InTypeName),
            true);
        if (!EnumPtr)
        {
            // INDEX_NONE
            return -1;
        }

        // Or GetIndexByName to return Index
        return EnumPtr->GetValueByName(FName(*InEnumStringValue));
    }

    FORCEINLINE static FString GetWorldTypeAsString(const EWorldType::Type InWorldType)
    {
        switch (InWorldType)
        {
            case EWorldType::None:
                return TEXT("None");
            case EWorldType::Game:
                return TEXT("Game");
            case EWorldType::Editor:
                return TEXT("Editor");
            case EWorldType::PIE:
                return TEXT("PIE");
            case EWorldType::EditorPreview:
                return TEXT("EditorPreview");
            case EWorldType::GamePreview:
                return TEXT("GamePreview");
            case EWorldType::GameRPC:
                return TEXT("GameRPC");
            case EWorldType::Inactive:
                return TEXT("Inactive");
            default:
                return FString();
        }
    }

    FORCEINLINE static FString GetERRResourceDataTypeAsString(const ERRResourceDataType InDataType)
    {
        return GetEnumValueAsString("ERRResourceDataType", InDataType);
    }
};
