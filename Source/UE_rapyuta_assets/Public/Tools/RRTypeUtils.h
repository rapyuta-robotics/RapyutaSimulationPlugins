#pragma once

// UE
#include "AssetRegistryModule.h"
#include "Kismet/BlueprintFunctionLibrary.h"

// RapyutaSim
#include "RRGameSingleton.h"
#include "RRObjectCommon.h"
#include "UE_rapyuta_assets.h"

#include "RRTypeUtils.generated.h"

UCLASS()
class UE_RAPYUTA_ASSETS_API URRTypeUtils : public UBlueprintFunctionLibrary
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
