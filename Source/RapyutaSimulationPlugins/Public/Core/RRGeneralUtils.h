// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#pragma once

// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "TimerManager.h"

#include "RRGeneralUtils.generated.h"

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRGeneralUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    static bool GetRefTransform(const FString& RefActorName, const AActor* RefActor, FTransform& OutTransf)
    {
        if (RefActorName.IsEmpty())    // refrence is world origin
        {
            OutTransf = FTransform::Identity;
        }
        else
        {
            if (RefActor == nullptr)
            {
                return false;
            }
            OutTransf = RefActor->GetTransform();
        }
        return true;
    }

    static FTransform GetRelativeTransform(const FTransform& RefTransf, const FTransform& WorldTransf)
    {
        FTransform refTransfNormalized = RefTransf;
        refTransfNormalized.NormalizeRotation();

        FTransform relativeTransf = WorldTransf.GetRelativeTransform(refTransfNormalized);
        relativeTransf.NormalizeRotation();

        return relativeTransf;
    }

    static bool GetRelativeTransform(const FString& RefActorName,
                                     const AActor* RefActor,
                                     const FTransform& InTransf,
                                     FTransform& OutTransf)
    {
        FTransform refTransf;
        bool result = GetRefTransform(RefActorName, RefActor, refTransf);
        if (result)
        {
            OutTransf = URRGeneralUtils::GetRelativeTransform(refTransf, InTransf);
        }
        return result;
    }

    static FTransform GetWorldTransform(const FTransform& RefTransf, const FTransform& RelativeTransf)
    {
        FTransform worldTransf;

        FTransform::Multiply(&worldTransf, &RelativeTransf, &RefTransf);

        worldTransf.NormalizeRotation();

        return worldTransf;
    }

    static bool GetWorldTransform(const FString& RefActorName,
                                  const AActor* RefActor,
                                  const FTransform& InTransf,
                                  FTransform& OutTransf)
    {
        FTransform refTransf;
        bool result = GetRefTransform(RefActorName, RefActor, refTransf);
        if (result)
        {
            OutTransf = URRGeneralUtils::GetWorldTransform(refTransf, InTransf);
        }
        return result;
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
