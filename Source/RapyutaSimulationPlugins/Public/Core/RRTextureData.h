// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"

#include "RRTextureData.generated.h"

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRTextureData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    TArray<UTexture*> ImageTextureList;

    UPROPERTY()
    TArray<FString> TextureNames;

    UTexture* GetRandomTexture() const;
};
