// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "Engine/EngineTypes.h"
#include "GameFramework/GameMode.h"

// RapyutaSimulationPlugins

#include "RRROS2GameMode.generated.h"

class AROS2Node;
class URRROS2ClockPublisher;
class ASimulationState;
UCLASS() class RAPYUTASIMULATIONPLUGINS_API ARRROS2GameMode : public AGameMode
{
    GENERATED_BODY()

public:
    ARRROS2GameMode();

    UFUNCTION(BlueprintCallable)
    virtual void Init();

    UPROPERTY(BlueprintReadWrite)
    AROS2Node* ROS2Node = nullptr;

    UPROPERTY(BlueprintReadWrite)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    UPROPERTY(BlueprintReadWrite)
    ASimulationState* SimulationState = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString UENodeName = TEXT("UENode");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bAutoStart = true;


protected:
    virtual void BeginPlay() override;
};
