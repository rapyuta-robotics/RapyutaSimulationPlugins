// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/TimeLogger.h"

// Sets default values
ATimeLogger::ATimeLogger()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ATimeLogger::BeginPlay()
{
    Super::BeginPlay();

    // 1 minute worth of data at 200 FPS
    RealTimeHistory.Reserve(1000);
    SimTimeHistory.Reserve(1000);
}

// Called every frame
void ATimeLogger::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    FDateTime CurrentRealTime = FDateTime::Now();
    UWorld* world = GetWorld();
    float CurrentRealTime_s =
        CurrentRealTime.GetMinute() * 60.f + CurrentRealTime.GetSecond() + CurrentRealTime.GetMillisecond() * .001f;
    float CurrentSimTime = UGameplayStatics::GetRealTimeSeconds(world);    // clock
    RealTimeHistory.Add(FString::SanitizeFloat(CurrentRealTime_s));
    SimTimeHistory.Add(FString::SanitizeFloat(CurrentSimTime));

    if (CurrentSimTime - StartSimTime >= MaxTime)
    {
        DumpData();
        UKismetSystemLibrary::QuitGame(world, nullptr, EQuitPreference::Quit, true);
    }
}

void ATimeLogger::StartTimer()
{
    StartSimTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    StartRealTime = FDateTime::Now();
}

void ATimeLogger::DumpData()
{
    FString Directory = FPaths::ProjectContentDir();
    FString TargetFile_RealTime = Directory + "/RealTime_UE4";
    FString TargetFile_SimTime = Directory + "/SimTime_UE4";

    FFileHelper::SaveStringArrayToFile(RealTimeHistory, *TargetFile_RealTime);
    FFileHelper::SaveStringArrayToFile(SimTimeHistory, *TargetFile_SimTime);
}
