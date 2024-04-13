/**
 * @file RRGeneralUtils.h
 * @brief General utils.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Json.h"
#include "Kismet/KismetSystemLibrary.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "TimerManager.h"

// #include "Core/RRUObjectUtils.h"

#include "RRGeneralUtils.generated.h"

// NOTE: Using TCHAR* = TEXT("") -> could cause linking error in some case!
#define EMPTY_STR (TEXT(""))

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
     * @brief Find actor by name. GetAllActors() is expensive.
     *
     * @tparam T
     * @param InWorld
     * @param InName
     * @param InCaseType
     * @return T*
     *
     */
    template<typename T>
    static T* FindActorByName(UWorld* InWorld, const FString& InName, const ESearchCase::Type InCaseType = ESearchCase::IgnoreCase)
    {
        for (TActorIterator<T> actorItr(InWorld); actorItr; ++actorItr)
        {
            if (actorItr->GetName().Equals(InName, InCaseType))
            {
                return *actorItr;
            }
        }
#if WITH_EDITOR
        // check Display Name if actor not found by ID Name
        for (TActorIterator<T> actorItr(InWorld); actorItr; ++actorItr)
        {
            if (UKismetSystemLibrary::GetDisplayName(*actorItr).Equals(InName, InCaseType))
            {
                return *actorItr;
            }
        }
#endif
        UE_LOG(LogTemp, Log, TEXT("Actor named [%s] is unavailable."), *InName);
        return nullptr;
    }

    /**
     * @brief Blueprint Callable, non template version of FindActorByName
     *
     * @param WorldContextObject
     * @param InName
     * @param InCaseType
     * @return AActor*
     */
    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static AActor* FindActorByName(const UObject* WorldContextObject,
                                   const FString& InName,
                                   const ESearchCase::Type InCaseType = ESearchCase::IgnoreCase)
    {
        UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
        return FindActorByName<AActor>(World, InName, InCaseType);
    }

    /**
     * @brief Find actor by subname. search actor whose name contains InSubname.
     *
     * @tparam T
     * @param InWorld
     * @param InSubname
     * @param InCaseType
     * @return T*
     */
    template<typename T>
    static T* FindActorBySubname(UWorld* InWorld,
                                 const FString& InSubname,
                                 const ESearchCase::Type InCaseType = ESearchCase::IgnoreCase)
    {
        for (TActorIterator<T> actorItr(InWorld); actorItr; ++actorItr)
        {
            if (actorItr->GetName().Contains(InSubname, InCaseType))
            {
                return *actorItr;
            }
        }
#if WITH_EDITOR
        // check Display Name if actor not found by ID Name
        for (TActorIterator<T> actorItr(InWorld); actorItr; ++actorItr)
        {
            if (UKismetSystemLibrary::GetDisplayName(*actorItr).Contains(InSubname, InCaseType))
            {
                return *actorItr;
            }
        }
#endif
        UE_LOG(LogTemp, Log, TEXT("Actor name containing [%s] is unavailable."), *InSubname);
        return nullptr;
    }

    /**
     * @brief Blueprint Callable, non template version of FindActorBySubname
     *
     * @param WorldContextObject
     * @param InSubname
     * @param InCaseType
     * @return AActor*
     */
    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static AActor* FindActorBySubname(const UObject* WorldContextObject,
                                      const FString& InSubname,
                                      const ESearchCase::Type InCaseType = ESearchCase::IgnoreCase)
    {
        UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
        return FindActorBySubname<AActor>(World, InSubname, InCaseType);
    }

    template<typename T>
    static TArray<T*> FindActorListBySubname(UWorld* InWorld,
                                             const FString& InSubname,
                                             const ESearchCase::Type InCaseType = ESearchCase::IgnoreCase)
    {
        TArray<T*> actors;
        for (TActorIterator<T> actorItr(InWorld); actorItr; ++actorItr)
        {
            if (actorItr->GetName().Contains(InSubname, InCaseType))
            {
                actors.Add(*actorItr);
            }
#if WITH_EDITOR
            // check Display Name if actor not found by ID Name
            else if (UKismetSystemLibrary::GetDisplayName(*actorItr).Contains(InSubname, InCaseType))
            {
                actors.Add(*actorItr);
            }
#endif
        }

        return actors;
    }

    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static TArray<AActor*> FindActorListBySubname(const UObject* WorldContextObject,
                                                  const FString& InSubname,
                                                  const ESearchCase::Type InCaseType = ESearchCase::IgnoreCase)
    {
        UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
        return FindActorListBySubname<AActor>(World, InSubname, InCaseType);
    }

    /**
     * @brief Get the Ref Transform.
     * @param RefActor  Reference Actor
     * @param OutTransf Transform of RefActor or Identity.
     * @param ReturnIdentityWithNullptr If this is true, return true and OutTransf = FTransform::Identity with nullptr RefActor.
     * @param Verbose
     * @return true
     * @return false
     */
    static bool GetRefTransform(const AActor* RefActor,
                                FTransform& OutTransf,
                                const bool ReturnIdentityWithNullptr = true,
                                const bool Verbose = false)
    {
        bool res = false;
        if (RefActor == nullptr)
        {
            if (ReturnIdentityWithNullptr)
            {
                OutTransf = FTransform::Identity;
                res = true;
            }
            else
            {
                if (Verbose)
                {
                    UE_LOG(LogTemp, Error, TEXT("RefActor is not valid."));
                }
                res = false;
            }
        }
        else
        {
            OutTransf = RefActor->GetTransform();
            res = true;
        }
        return res;
    }

    /**
     * @brief Get the Ref Transform. If RefActor==nullptr, search actor from RefActorName. If no actor is found, return false and OutTransf = FTransform::Identity
     * If RefActor==nullptr && RefActorName==Empty, return true and OutTransf = FTransform::Identity
     * @param RefActorName
     * @param RefActor
     * @param InWorld
     * @param OutTransf
     * @param Verbose
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static bool GetRefTransform(const FString& RefActorName,
                                const AActor* RefActor,
                                const UObject* WorldContextObject,
                                FTransform& OutTransf,
                                const bool Verbose = false)
    {
        bool res = GetRefTransformByActor(RefActor, OutTransf, Verbose);
        if (!res)
        {
            res = GetRefTransformByName(RefActorName, WorldContextObject, OutTransf, Verbose);
        }
        return res;
    }

    /**
     * @brief Get the Ref Transform. If RefActor==nullptr, OutTransf = FTransform::Identity
     *
     * @param RefActor
     * @param OutTransf
     * @param Verbose
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable)
    static bool GetRefTransformByActor(const AActor* RefActor, FTransform& OutTransf, const bool Verbose = false)
    {
        if (RefActor == nullptr)
        {
            OutTransf = FTransform::Identity;
            if (Verbose)
            {
                UE_LOG(LogTemp, Error, TEXT("RefActor is not valid."));
            }
            return false;
        }
        OutTransf = RefActor->GetTransform();
        return true;
    }

    /**
     * @brief Get the Ref Transform. Search actor from name. If no actor is found, return false and OutTransf = FTransform::Identity
     *
     * @param RefActorName
     * @param WorldContextObject
     * @param OutTransf
     * @param Verbose
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static bool GetRefTransformByName(const FString& RefActorName,
                                      const UObject* WorldContextObject,
                                      FTransform& OutTransf,
                                      const bool Verbose = false)
    {
        if (RefActorName.IsEmpty())    // refrence is world origin
        {
            OutTransf = FTransform::Identity;
            return true;
        }

        if (WorldContextObject == nullptr)
        {
            OutTransf = FTransform::Identity;
            if (Verbose)
            {
                UE_LOG(LogTemp, Error, TEXT("World is not given. Return Idnetity Transform"));
            }
            return false;
        }

        AActor* refActor = URRGeneralUtils::FindActorByName(WorldContextObject, RefActorName);
        if (refActor == nullptr)
        {
            OutTransf = FTransform::Identity;
            if (Verbose)
            {
                UE_LOG(LogTemp, Warning, TEXT("Reference Actor %s is not valid."), *RefActorName);
            }
            return false;
        }

        OutTransf = refActor->GetTransform();
        return true;
    }

    /**
     * @brief Get the transform in reference frame.
     *
     * @param RefTransf Reference frame
     * @param WorldTransf Transform in world frame
     * @param IgnoreScale
     * @return FTransform Transform in reference frame
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetRelativeTransform(const FTransform& RefTransf,
                                           const FTransform& WorldTransf,
                                           const bool IgnoreScale = false)
    {
        FTransform worldTransf = WorldTransf;
        FTransform refTransfNormalized = RefTransf;
        refTransfNormalized.NormalizeRotation();
        if (IgnoreScale)
        {
            worldTransf.SetScale3D(FVector::OneVector);
            refTransfNormalized.SetScale3D(FVector::OneVector);
        }

        FTransform relativeTransf = worldTransf.GetRelativeTransform(refTransfNormalized);
        relativeTransf.NormalizeRotation();

        return relativeTransf;
    }

    /**
     * @brief Get the transform in reference frame. If RefActor==nullptr, return WorldTransf
     *
     * @param RefActor
     * @param WorldTransf Transform in world frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in reference frame
     */
    static FTransform GetRelativeTransform(const AActor* RefActor,
                                           const FTransform& WorldTransf,
                                           const bool IgnoreScale = false,
                                           const bool Verbose = false)
    {
        FTransform outTransf;
        GetRefTransformByActor(RefActor, outTransf, Verbose);
        return GetRelativeTransform(outTransf, WorldTransf, IgnoreScale);
    }

    /**
     * @brief Get the transform in reference frame.  If Actor with given name is not exists, return WorldTransf
     *
     * @param RefActorName
     * @param WorldContextObject
     * @param WorldTransf Transform in world frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in reference frame
     */
    static FTransform GetRelativeTransform(const FString& RefActorName,
                                           const UObject* WorldContextObject,
                                           const FTransform& WorldTransf,
                                           const bool IgnoreScale = false,
                                           const bool Verbose = false)
    {
        FTransform outTransf;
        GetRefTransformByName(RefActorName, WorldContextObject, outTransf, Verbose);
        return GetRelativeTransform(outTransf, WorldTransf, IgnoreScale);
    }

    /**
     * @brief Blueprint wrapper for GetRelativeTransform
     *
     * @param RefActor
     * @param WorldTransf Transform in world frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in reference frame
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetRelativeTransformFromActor(const AActor* RefActor,
                                                    const FTransform& WorldTransf,
                                                    const bool IgnoreScale = false,
                                                    const bool Verbose = false)
    {
        return GetRelativeTransform(RefActor, WorldTransf, IgnoreScale, Verbose);
    }

    /**
     * @brief Blueprint wrapper for GetRelativeTransform
     *
     * @param RefActor
     * @param WorldContextObject
     * @param WorldTransf Transform in world frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in reference frame
     */
    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static FTransform GetRelativeTransformFromName(const FString& RefActorName,
                                                   const UObject* WorldContextObject,
                                                   const FTransform& WorldTransf,
                                                   const bool IgnoreScale = false,
                                                   const bool Verbose = false)
    {
        return GetRelativeTransform(RefActorName, WorldContextObject, WorldTransf, IgnoreScale, Verbose);
    }

    /**
     * @brief Get the transform in reference frame.
     * @param RefActor  Reference Actor
     * @param InTransf
     * @param OutTransf Transform of RefActor or Identity.
     * @param IgnoreScale
     * @param ReturnIdentityWithNullptr if this is true, return true and OutTransf = FTransform::Identity with nullptr RefActor.
     * @param Verbose
     * @return true
     * @return false
     */
    static bool GetRelativeTransform(const AActor* RefActor,
                                     const FTransform& InTransf,
                                     FTransform& OutTransf,
                                     const bool IgnoreScale = false,
                                     const bool ReturnIdentityWithNullptr = true,
                                     const bool Verbose = false)
    {
        FTransform refTransf;
        bool result = GetRefTransform(RefActor, refTransf, ReturnIdentityWithNullptr, Verbose);
        if (result)
        {
            OutTransf = GetRelativeTransform(refTransf, InTransf, IgnoreScale);
        }
        return result;
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
                                     const UObject* WorldContextObject,
                                     FTransform& OutTransf,
                                     const bool IgnoreScale = false,
                                     const bool Verbose = false)
    {
        FTransform refTransf;
        bool result = GetRefTransform(RefActorName, RefActor, WorldContextObject, refTransf, Verbose);
        if (result)
        {
            OutTransf = GetRelativeTransform(refTransf, InTransf, IgnoreScale);
        }
        return result;
    }

    /**
     * @brief Get the transform in world frame
     *
     * @param RefTransf Reference frame
     * @param RelativeTransf Transform in reference frame
     * @param IgnoreScale
     * @return FTransform Transform in world frame
     */
    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static FTransform GetWorldTransform(const FTransform& RefTransf,
                                        const FTransform& RelativeTransf,
                                        const bool IgnoreScale = false)
    {
        FTransform worldTransf;
        FTransform refTransf = RefTransf;
        FTransform relativeTransf = RelativeTransf;
        if (IgnoreScale)
        {
            refTransf.SetScale3D(FVector::OneVector);
            relativeTransf.SetScale3D(FVector::OneVector);
        }

        FTransform::Multiply(&worldTransf, &relativeTransf, &refTransf);

        worldTransf.NormalizeRotation();

        return worldTransf;
    }

    /**
     * @brief Get the transform in world frame. If RefActor==nullptr, return RelativeTransf
     *
     * @param RefActor
     * @param RelativeTransf Transform in reference frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in world frame
     */
    static FTransform GetWorldTransform(const AActor* RefActor,
                                        const FTransform& RelativeTransf,
                                        const bool IgnoreScale = false,
                                        const bool Verbose = false)
    {
        FTransform outTransf;
        GetRefTransformByActor(RefActor, outTransf, Verbose);
        return GetWorldTransform(outTransf, RelativeTransf, IgnoreScale);
    }

    /**
     * @brief Get the transform in world frame. If RefActor==nullptr, return RelativeTransf
     *
     * @param RefActorName
     * @param WorldContextObject
     * @param RefActor
     * @param RelativeTransf Transform in reference frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in world frame
     */
    static FTransform GetWorldTransform(const FString& RefActorName,
                                        const UObject* WorldContextObject,
                                        const FTransform& RelativeTransf,
                                        const bool IgnoreScale = false,
                                        const bool Verbose = false)
    {
        FTransform outTransf;
        GetRefTransformByName(RefActorName, WorldContextObject, outTransf, Verbose);
        return GetWorldTransform(outTransf, RelativeTransf, IgnoreScale);
    }

    /**
     * @brief Blueprint wrapper for GetRelativeTransform
     *
     * @param RefActor
     * @param RelativeTransf Transform in reference frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in world frame
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetWorldTransformFromActor(const AActor* RefActor,
                                                 const FTransform& RelativeTransf,
                                                 const bool IgnoreScale = false,
                                                 const bool Verbose = false)
    {
        return GetWorldTransform(RefActor, RelativeTransf, IgnoreScale, Verbose);
    }

    /**
     * @brief Blueprint wrapper for GetRelativeTransform
     *
     * @param RefActorName
     * @param WorldContextObject
     * @param RelativeTransf Transform in reference frame
     * @param IgnoreScale
     * @param Verbose
     * @return FTransform Transform in world frame
     */
    UFUNCTION(BlueprintCallable, meta = (WorldContext = WorldContextObject))
    static FTransform GetWorldTransformFromName(const FString& RefActorName,
                                                const UObject* WorldContextObject,
                                                const FTransform& RelativeTransf,
                                                const bool IgnoreScale = false,
                                                const bool Verbose = false)
    {
        return GetWorldTransform(RefActorName, WorldContextObject, RelativeTransf, IgnoreScale, Verbose);
    }

    /**
     * @brief Get the transform in world frame
     *
     * @param RefActorName If this is empty, use world origin as reference, i.e. OutTransf=InTransf
     * @param RefActor If this is nullptr, return false.
     * @param InTransf Transform in reference frame
     * @param OutTransf Transform in world frame
     * @param ReturnIdentityWithNullptr
     * @param IgnoreScale
     * @param Verbose
     * @return true
     * @return false
     */
    static bool GetWorldTransform(const AActor* RefActor,
                                  const FTransform& InTransf,
                                  FTransform& OutTransf,
                                  const bool ReturnIdentityWithNullptr = true,
                                  const bool IgnoreScale = false,
                                  const bool Verbose = false)
    {
        FTransform refTransf;
        bool result = GetRefTransform(RefActor, refTransf, ReturnIdentityWithNullptr, Verbose);
        if (result)
        {
            OutTransf = GetWorldTransform(refTransf, InTransf, IgnoreScale);
        }
        return result;
    }

    /**
     * @brief Get the transform in world frame
     *
     * @param RefActorName If this is empty, use world origin as reference, i.e. OutTransf=InTransf
     * @param RefActor If this is nullptr, return false.
     * @param InTransf Transform in reference frame
     * @param OutTransf Transform in world frame
     * @param IgnoreScale
     * @return true
     * @return false
     */
    static bool GetWorldTransform(const FString& RefActorName,
                                  const AActor* RefActor,
                                  const FTransform& InTransf,
                                  const UObject* WorldContextObject,
                                  FTransform& OutTransf,
                                  const bool IgnoreScale = false)
    {
        FTransform refTransf;
        bool result = GetRefTransform(RefActorName, RefActor, WorldContextObject, refTransf);
        if (result)
        {
            OutTransf = GetWorldTransform(refTransf, InTransf, IgnoreScale);
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

    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj, const FString& InFieldName, FString& OutValue)
    {
        return InJsonObj.Get()->TryGetStringField(InFieldName, OutValue);
    }
    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @param InMultiplier (optional) returned value is multiplied by this. Set to 1.f by default
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj,
                                         const FString& InFieldName,
                                         float& OutValue,
                                         float InMultiplier = 1.f)
    {
        double resultValue;
        bool bFieldFound = InJsonObj.Get()->TryGetNumberField(InFieldName, resultValue);
        if (!bFieldFound)
        {
            return false;
        }
        OutValue = static_cast<float>(resultValue) * InMultiplier;
        return true;
    }
    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @param InMultiplier (optional) returned value is multiplied by this. Set to 1.f by default
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj,
                                         const FString& InFieldName,
                                         double& OutValue,
                                         double InMultiplier = 1.)
    {
        bool bFieldFound = InJsonObj.Get()->TryGetNumberField(InFieldName, OutValue);
        if (!bFieldFound)
        {
            return false;
        }
        OutValue *= InMultiplier;
        return true;
    }
    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj, const FString& InFieldName, int& OutValue)
    {
        return InJsonObj.Get()->TryGetNumberField(InFieldName, OutValue);
    }
    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj, const FString& InFieldName, bool& OutValue)
    {
        return InJsonObj.Get()->TryGetBoolField(InFieldName, OutValue);
    }

    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj, const FString& InFieldName, FVector& OutValue)
    {
        bool res = true;
        auto const tempJsonObj = InJsonObj->GetObjectField(InFieldName);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("x"), OutValue.X);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("y"), OutValue.Y);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("z"), OutValue.Z);

        return res;
    }

    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj, const FString& InFieldName, FRotator& OutValue)
    {
        bool res = true;
        auto const tempJsonObj = InJsonObj->GetObjectField(InFieldName);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("roll"), OutValue.Roll);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("pitch"), OutValue.Pitch);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("yaw"), OutValue.Yaw);

        return res;
    }

    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj, const FString& InFieldName, FQuat& OutValue)
    {
        bool res = true;
        auto const tempJsonObj = InJsonObj->GetObjectField(InFieldName);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("x"), OutValue.X);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("y"), OutValue.Y);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("z"), OutValue.Z);
        res &= tempJsonObj.Get()->TryGetNumberField(TEXT("w"), OutValue.W);

        return res;
    }

    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    FORCEINLINE static bool GetJsonField(const TSharedPtr<FJsonObject>& InJsonObj, const FString& InFieldName, FTransform& OutValue)
    {
        bool res = true;
        FVector vectorParam = FVector::ZeroVector;
        FRotator rotatorParam = FRotator::ZeroRotator;
        auto const tempJsonObj = InJsonObj->GetObjectField(InFieldName);
        res &= GetJsonField(tempJsonObj, TEXT("position"), vectorParam);
        res &= GetJsonField(tempJsonObj, TEXT("orientation"), rotatorParam);
        OutValue = FTransform(rotatorParam, vectorParam, FVector::OneVector);

        return res;
    }

    /**
     * @brief Initialize OutValue with the value of the requested field in a FJsonObject.
     * If the field does not exist, OutValue = InDefaultValue
     *
     * @param InJsonObj the Json object containing the required field
     * @param InFieldName the name of the field to read
     * @param InDefaultValue the value sent back if the field is not in the Json object
     * @param OutValue contains the returned value
     * @return bool if the field exists in the Json object
     */
    template<typename T>
    FORCEINLINE static bool GetJsonFieldOrDefault(const TSharedPtr<FJsonObject>& InJsonObj,
                                                  const FString& InFieldName,
                                                  const T& InDefaultValue,
                                                  T& OutValue)
    {
        if (GetJsonField(InJsonObj, InFieldName, OutValue))
        {
            return true;
        }
        OutValue = InDefaultValue;
        return false;
    }

    /**
     * @brief Get the component of actor from component name
     *
     * @param Actor
     * @param ComponentName
     * @return UPrimitiveComponent*
     */
    UFUNCTION(BlueprintCallable)
    static UPrimitiveComponent* GetComponentOfActorFromName(const AActor* Actor, FName ComponentName)
    {
        UPrimitiveComponent* PrimComp = NULL;

        if (Actor != NULL)
        {
            // No name specified, use the root component
            if (ComponentName == NAME_None)
            {
                PrimComp = Cast<UPrimitiveComponent>(Actor->GetRootComponent());
            }
            // Name specified, see if we can find that component..
            else
            {
                for (UActorComponent* Comp : Actor->GetComponents())
                {
                    if (Comp->GetFName() == ComponentName)
                    {
                        if (UChildActorComponent* ChildActorComp = Cast<UChildActorComponent>(Comp))
                        {
                            if (AActor* ChildActor = ChildActorComp->GetChildActor())
                            {
                                PrimComp = Cast<UPrimitiveComponent>(ChildActor->GetRootComponent());
                            }
                        }
                        else
                        {
                            PrimComp = Cast<UPrimitiveComponent>(Comp);
                        }
                        break;
                    }
                }
            }
        }

        return PrimComp;
    }

    /**
     * @brief Get the Physics Constraint Component.
     * @sa [EConstraintFrame](https://docs.unrealengine.com/5.0/en-US/API/Runtime/PhysicsCore/Chaos/EConstraintFrame__Type/)
     *
     * @param InConstraint
     * @param Frame
     * @return UPrimitiveComponent*
     */
    UFUNCTION(BlueprintCallable)
    static UPrimitiveComponent* GetPhysicsConstraintComponent(const UPhysicsConstraintComponent* InConstraint,
                                                              EConstraintFrame::Type Frame)
    {
        if (InConstraint != nullptr)
        {
            UPrimitiveComponent* PrimComp = NULL;

            FName ComponentName = NAME_None;
            AActor* Actor = NULL;

            // Frame 1
            if (Frame == EConstraintFrame::Frame1)
            {
                // Use override component if specified
                if (InConstraint->OverrideComponent1.IsValid())
                {
                    return InConstraint->OverrideComponent1.Get();
                }

                ComponentName = InConstraint->ComponentName1.ComponentName;
                Actor = InConstraint->ConstraintActor1;
            }
            // Frame 2
            else
            {
                // Use override component if specified
                if (InConstraint->OverrideComponent2.IsValid())
                {
                    return InConstraint->OverrideComponent2.Get();
                }

                ComponentName = InConstraint->ComponentName2.ComponentName;
                Actor = InConstraint->ConstraintActor2;
            }

            return GetComponentOfActorFromName(Actor, ComponentName);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[GetPhysicsConstraintComponent]Physics Constraint is not valid."));
            return nullptr;
        }
    }

    /**
     * @brief Get the Physics Constraint Transform changes from initial joint transform, i.e. child link transfrom relative to joint.
     *
     * @param InConstraint
     * @param InitialJointToChildLink
     * @param InChildLink
     * @return FTransform
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetPhysicsConstraintTransform(const UPhysicsConstraintComponent* InConstraint,
                                                    const FTransform InitialJointToChildLink,
                                                    UPrimitiveComponent* InChildLink = nullptr)
    {
        FTransform outTF = FTransform::Identity;
        if (InConstraint != nullptr)
        {
            UPrimitiveComponent* ChildLink = InChildLink;
            if (ChildLink == nullptr)
            {
                ChildLink = GetPhysicsConstraintComponent(InConstraint, EConstraintFrame::Frame2);
            }

            if (ChildLink != nullptr)
            {
                FTransform relativeTrans = URRGeneralUtils::GetRelativeTransform(InConstraint->GetComponentTransform(),
                                                                                 ChildLink->GetComponentTransform());

                FVector position = relativeTrans.GetLocation() - InitialJointToChildLink.GetLocation();
                FRotator orientation = (relativeTrans.GetRotation() * InitialJointToChildLink.GetRotation().Inverse()).Rotator();

                outTF.SetLocation(position);
                outTF.SetRotation(orientation.Quaternion());
            }
            else
            {
                outTF = FTransform::Identity;
            }
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[GetPhysicsConstraintTransform]Physics Constraint is not valid."));
            outTF = FTransform::Identity;
        }

        return outTF;
    }

    /**
     * @brief Get the Physics Constraint Transform
     *
     * @param InConstraint
     * @param InitialJointToChildLink
     * @param OutPosition
     * @param OutOrientation
     * @param InChildLink
     */
    static void GetPhysicsConstraintTransform(const UPhysicsConstraintComponent* InConstraint,
                                              const FTransform InitialJointToChildLink,
                                              FVector& OutPosition,
                                              FRotator& OutOrientation,
                                              UPrimitiveComponent* InChildLink = nullptr)
    {
        FTransform tf = GetPhysicsConstraintTransform(InConstraint, InitialJointToChildLink, InChildLink);
        OutPosition = tf.GetLocation();
        OutOrientation = tf.GetRotation().Rotator();
    }

    UFUNCTION(BlueprintCallable, BlueprintPure)
    static FString PascalToSnake(const FString& InPascalString, const bool InCheckNum = false)
    {
        FString output = TEXT("");
        for (int32 i = 0; i < InPascalString.Len(); i++)
        {
            FString currStr = InPascalString.Mid(i, 1);
            FString newStr = currStr;
            if (i > 0 && (isupper(*TCHAR_TO_ANSI(*currStr)) || (InCheckNum && currStr.IsNumeric())))
            {
                newStr = TEXT("_") + newStr.ToLower();
            }
            else
            {
                newStr = newStr.ToLower();
            }
            output.Append(newStr);
        }
        return output;
    }

    UFUNCTION(BlueprintCallable, BlueprintPure)
    static USceneComponent* FindChildComponentByClass(const USceneComponent* InTarget,
                                                      const TSubclassOf<UActorComponent> InComponentClass,
                                                      bool bIncludeAllDescendants = false)
    {
        TArray<USceneComponent*> children;
        InTarget->GetChildrenComponents(bIncludeAllDescendants, children);
        for (const auto& child : children)
        {
            if (child->IsA(InComponentClass))
            {
                return child;
            }
        }
        return nullptr;
    }
};
