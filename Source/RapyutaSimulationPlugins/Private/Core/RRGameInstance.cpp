// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "Core/RRGameInstance.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameMode.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMathUtils.h"

void URRGameInstance::PreloadContentForURL(FURL InURL)
{
    UE_LOG(LogRapyutaCore, Display, TEXT("[URRGameInstance]: Preload Content for Map [%s]"), *InURL.Map)
    Super::PreloadContentForURL(InURL);
}

AGameModeBase* URRGameInstance::CreateGameModeForURL(FURL InURL, UWorld* InWorld)
{
    UE_LOG(LogRapyutaCore,
           Display,
           TEXT("[URRGameInstance]: URL MAP PATH [%s] loaded into WORLD [%s]"),
           *InURL.Map,
           *InWorld->GetName());

    InitialMapName = InWorld->GetName();
    URRCoreUtils::ScreenMsg(FColor::Yellow, FString::Printf(TEXT("MAP: %s"), *InitialMapName));
    return Super::CreateGameModeForURL(InURL, InWorld);
}

void URRGameInstance::StartGameInstance()
{
    UE_LOG(LogRapyutaCore, Display, TEXT("URRGameInstance: StartGameInstance()!"));
    Super::StartGameInstance();
}

void URRGameInstance::Init()
{
    UE_LOG(LogRapyutaCore, Display, TEXT("URRGameInstance: INIT!"))
    Super::Init();
}

void URRGameInstance::LoadComplete(const float InLoadTime, const FString& InMapName)
{
    Super::LoadComplete(InLoadTime, InMapName);
    UE_LOG(LogRapyutaCore, Display, TEXT("URRGameInstance: [%s] LOADING COMPLETED!"), *InMapName);
}

void URRGameInstance::OnStart()
{
    UE_LOG(LogRapyutaCore, Display, TEXT("URRGameInstance: ON START!"))
    Super::OnStart();
}
