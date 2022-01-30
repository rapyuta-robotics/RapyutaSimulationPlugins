// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#pragma once

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


    static FTransform RelativeTransform(const FTransform& RefTrans, const FTransform& WorldTrans)
    {
        FTransform RelativeTrans;

        FQuat RefQuatInv = RefTrans.GetRotation().Inverse();

        RelativeTrans.SetTranslation(RefQuatInv.RotateVector(WorldTrans.GetTranslation() - RefTrans.GetTranslation()));
        RelativeTrans.SetRotation(WorldTrans.GetRotation() * RefQuatInv);

        RelativeTrans.NormalizeRotation();

        return RelativeTrans;
    }

    static FTransform WorldTransform(const FTransform& RefTrans, const FTransform& RelativeTrans)
    {
        FTransform WorldTrans;
        
        FQuat RefQuat = RefTrans.GetRotation();

        WorldTrans.SetTranslation(RefQuat.RotateVector(RelativeTrans.GetTranslation()) + RefTrans.GetTranslation());
        WorldTrans.SetRotation(RelativeTrans.GetRotation() * RefQuat);

        WorldTrans.NormalizeRotation();

        return WorldTrans;
    }

    FORCEINLINE static FString GetNewROS2NodeName(const FString& InAffix = FString())
    {
        return FString::Printf(TEXT("UE%s_%s"), *InAffix, *FGuid::NewGuid().ToString());
    }

    FORCEINLINE static FString ComposeROSFullFrameId(const FString& InPrefix, const TCHAR* InFrameId)
    {
        return InPrefix.IsEmpty() ? InFrameId : FString::Printf(TEXT("%s/%s"), *InPrefix, InFrameId);
    }
};
