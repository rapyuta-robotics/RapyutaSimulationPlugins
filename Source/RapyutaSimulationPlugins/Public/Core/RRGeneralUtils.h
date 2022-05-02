/**
 * @file RRGeneralUtils.h
 * @brief General utils.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */


#pragma once

// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "TimerManager.h"

#include "RRGeneralUtils.generated.h"

/**
 * @brief General utils
 * 
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRGeneralUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    /**
     * @brief Get the Ref Transform.
     * If RefActor==nullptr, return false.
     * @param RefActorName  If this is empty, OutTranf become FTransform::Identity, i.e. reference become world origin.
     * @param RefActor  If this is nullptr, return false.
     * @param OutTransf Transform of RefActor or Identity.
     * @return true 
     * @return false 
     */
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

    /**
     * @brief Get the transfrom in reference frame.
     * 
     * @param RefTransf Reference frame
     * @param WorldTransf Transform in world frame
     * @return FTransform Transform in reference frame
     */
    static FTransform GetRelativeTransform(const FTransform& RefTransf, const FTransform& WorldTransf)
    {
        FTransform refTransfNormalized = RefTransf;
        refTransfNormalized.NormalizeRotation();

        FTransform relativeTransf = WorldTransf.GetRelativeTransform(refTransfNormalized);
        relativeTransf.NormalizeRotation();

        return relativeTransf;
    }

    /**
     * @brief Get the transform in reference frame.
     * 
     * @param RefActorName If this is empty, use world origin as reference, i.e. OutTransf=InTransf
     * @param RefActor If this is nullptr, return false.
     * @param InTransf Transform in world frame
     * @param OutTransf Transform in reference frame
     * @return true 
     * @return false 
     */
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

    /**
     * @brief Get the transform in world frame
     * 
     * @param RefTransf Reference frame
     * @param RelativeTransf Transfrom in reference frame
     * @return FTransform Transform in world frame
     */
    static FTransform GetWorldTransform(const FTransform& RefTransf, const FTransform& RelativeTransf)
    {
        FTransform worldTransf;

        FTransform::Multiply(&worldTransf, &RelativeTransf, &RefTransf);

        worldTransf.NormalizeRotation();

        return worldTransf;
    }

    /**
     * @brief Get the transform in world frame
     * 
     * @param RefActorName If this is empty, use world origin as reference, i.e. OutTransf=InTransf
     * @param RefActor If this is nullptr, return false.
     * @param InTransf Transform in reference frame
     * @param OutTransf Transform in world frame
     *
     * @return true 
     * @return false 
     */
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

    /**
     * @brief Create Unique name start with UE + InAffix_ + Guid
     * 
     * @param InAffix 
     * @return FString Unique name
     */
    FORCEINLINE static FString GetNewROS2NodeName(const FString& InAffix = FString())
    {
        return FString::Printf(TEXT("UE%s_%s"), *InAffix, *FGuid::NewGuid().ToString());
    }

    /**
     * @brief Create prefixed frame_id
     * 
     * @param InPrefix 
     * @param InFrameId 
     * @return FString prefixed frame_id 
     */
    FORCEINLINE static FString ComposeROSFullFrameId(const FString& InPrefix, const TCHAR* InFrameId)
    {
        return InPrefix.IsEmpty() ? InFrameId : FString::Printf(TEXT("%s/%s"), *InPrefix, InFrameId);
    }
};
