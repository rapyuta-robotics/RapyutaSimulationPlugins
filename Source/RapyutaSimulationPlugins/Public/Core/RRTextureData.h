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

// Ref: DatasmithRuntime::FTextureData
struct RAPYUTASIMULATIONPLUGINS_API FRRLightProfileData
{
public:
    EPixelFormat PixelFormat = EPixelFormat::PF_Unknown;
    int32 Width = 0;
    int32 Height = 0;
    uint32 Pitch = 0;
    int16 BytesPerPixel = 0;
    FUpdateTextureRegion2D Region = {0, 0, 0, 0, 0, 0};
    uint8* ImageData = nullptr;
    // For IES profile
    float Brightness = -FLT_MAX;
    float TextureMultiplier = -FLT_MAX;
};
