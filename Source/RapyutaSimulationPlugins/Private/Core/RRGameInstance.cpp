// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRGameInstance.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMathUtils.h"

FString URRGameInstance::SMapName;
void URRGameInstance::PreloadContentForURL(FURL InURL)
{
    UE_LOG(LogRapyutaCore, Display, TEXT("Preload Content for Map [%s]"), *InURL.Map)
    Super::PreloadContentForURL(InURL);
}

AGameModeBase* URRGameInstance::CreateGameModeForURL(FURL InURL, UWorld* InWorld)
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT(" URL MAP PATH [%s] loaded into WORLD [%s]"), *InURL.Map, *InWorld->GetName());

    SMapName = InWorld->GetName();
    URRCoreUtils::ScreenMsg(FColor::Yellow, FString::Printf(TEXT("MAP: %s"), *SMapName));
    return Super::CreateGameModeForURL(InURL, InWorld);
}

void URRGameInstance::StartGameInstance()
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("StartGameInstance!"));
    Super::StartGameInstance();
}

void URRGameInstance::Init()
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("INIT!"))
    Super::Init();
    URRMathUtils::InitializeRandomStream();
}

void URRGameInstance::LoadComplete(const float InLoadTime, const FString& InMapName)
{
    Super::LoadComplete(InLoadTime, InMapName);
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("[%s] LOADING COMPLETED!"), *InMapName);
}

void URRGameInstance::OnStart()
{
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("ON START!"))
    Super::OnStart();
}
