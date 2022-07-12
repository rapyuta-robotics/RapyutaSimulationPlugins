// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

// UE
#include "CoreMinimal.h"
#include "GameFramework/GameStateBase.h"

#include "RRNetworkGameState.generated.h"

UCLASS() class RAPYUTASIMULATIONPLUGINS_API ARRNetworkGameState : public AGameState
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    ARRNetworkGameState();

protected:
    virtual float GetServerWorldTimeSeconds() const override;

};


