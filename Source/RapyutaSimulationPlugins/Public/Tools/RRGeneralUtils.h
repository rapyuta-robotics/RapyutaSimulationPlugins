// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

// UE
#include "Engine/World.h"
#include "TimerManager.h"

#include "RRGeneralUtils.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRGeneralUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    static bool CheckWithTimeOut(const TFunctionRef<bool()>& Condition,
                                 const TFunctionRef<void()>& Action,
                                 const FDateTime& BeginTime,
                                 float TimeoutInSec);

    static void StopRegisteredTimer(UWorld* World, FTimerHandle& TimerHandle)
    {
        // Also invalidate the timer here-in!
        World->GetTimerManager().ClearTimer(TimerHandle);
    }
};
