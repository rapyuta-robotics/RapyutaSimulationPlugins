// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Core/RRTextureData.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMathUtils.h"

UTexture* FRRTextureData::GetRandomTexture() const
{
    return (ImageTextureList.Num() > 0) ? URRMathUtils::GetRandomElement(ImageTextureList)
           : (TextureNames.Num() > 0)   ? URRGameSingleton::Get()->GetTexture(URRMathUtils::GetRandomElement(TextureNames), false)
                                        : nullptr;
}
