/**
 * @file RRTypeUtils.h
 * @brief UE type related utils
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "AssetRegistryModule.h"
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

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRTypeUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    // UE-TYPE RELATED UTILS ==
    //
    template<typename TEnum>
    FORCEINLINE static FString GetEnumValueAsString(const FString& InTypeName, TEnum InEnumValue)
    {
        const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, *InTypeName, true);
        if (!EnumPtr)
        {
            return "Invalid";
        }

        return EnumPtr->GetDisplayNameTextByIndex(static_cast<int32>(InEnumValue)).ToString();    // Or GetNameByValue
    }

    template<typename TEnum>
    FORCEINLINE static FString GetEnumNameByValue(const FString& InTypeName, TEnum InEnumValue)
    {
        const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, *InTypeName, true);
        if (!EnumPtr)
        {
            return "Invalid";
        }

        return EnumPtr->GetNameByValue(static_cast<int32>(InEnumValue)).ToString();    // Or GetNameByValue
    }

    FORCEINLINE static int8 GetEnumValueFromString(const FString& InTypeName, const FString& InEnumStringValue)
    {
        UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, *InTypeName, true);
        if (!EnumPtr)
        {
            // INDEX_NONE
            return -1;
        }

        // Or GetIndexByName to return Index
        return EnumPtr->GetValueByName(FName(*InEnumStringValue));
    }

    FORCEINLINE static FString GetERRResourceDataTypeAsString(const ERRResourceDataType InDataType)
    {
        return GetEnumValueAsString("ERRResourceDataType", InDataType);
    }
};
