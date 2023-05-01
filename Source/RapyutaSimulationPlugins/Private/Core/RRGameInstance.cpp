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
#if RAPYUTA_SIM_VERBOSE
    UE_LOG(LogRapyutaCore, Display, TEXT("Preload Content for Map [%s]"), *InURL.Map)
#endif
    Super::PreloadContentForURL(InURL);
}

AGameModeBase* URRGameInstance::CreateGameModeForURL(FURL InURL, UWorld* InWorld)
{
    SMapName = InWorld->GetName();
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT(" URL MAP PATH [%s] loaded into WORLD [%s]"), *InURL.Map, *InWorld->GetName());
    URRCoreUtils::ScreenMsg(FColor::Yellow, FString::Printf(TEXT("MAP: %s"), *SMapName));
#endif
    return Super::CreateGameModeForURL(InURL, InWorld);
}

void URRGameInstance::StartGameInstance()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("StartGameInstance!"));
#endif
    Super::StartGameInstance();
}

void URRGameInstance::Init()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("INIT!"))
#endif
    Super::Init();
    URRMathUtils::InitializeRandomStream();
}

void URRGameInstance::LoadComplete(const float InLoadTime, const FString& InMapName)
{
    Super::LoadComplete(InLoadTime, InMapName);
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("[%s] LOADING COMPLETED!"), *InMapName);
#endif
}

void URRGameInstance::OnStart()
{
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Display, TEXT("ON START!"))
#endif
    Super::OnStart();
}
