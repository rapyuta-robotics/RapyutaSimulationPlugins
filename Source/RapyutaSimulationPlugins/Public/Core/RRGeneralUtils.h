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
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "TimerManager.h"

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
     * @brief Get the Ref Transform.
     * If RefActor==nullptr, return false.
     * @param RefActorName  If this is empty, OutTranf become FTransform::Identity, i.e. reference become world origin.
     * @param RefActor  If this is nullptr, return false.
     * @param OutTransf Transform of RefActor or Identity.
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable)
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
     * @brief Get the transform in reference frame.
     *
     * @param RefTransf Reference frame
     * @param WorldTransf Transform in world frame
     * @return FTransform Transform in reference frame
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetRelativeTransform(const FTransform& RefTransf, const FTransform& WorldTransf)
    {
        FTransform refTransfNormalized = RefTransf;
        refTransfNormalized.NormalizeRotation();

        FTransform relativeTransf = WorldTransf.GetRelativeTransform(refTransfNormalized);
        relativeTransf.NormalizeRotation();

        return relativeTransf;
    }

    /**
     * @brief Get the transform in reference frame. If RefActor==nullptr, return WorldTransf
     *
     * @param RefActor
     * @param WorldTransf Transform in world frame
     * @return FTransform Transform in reference frame
     */
    static FTransform GetRelativeTransform(const AActor* RefActor, const FTransform& WorldTransf)
    {
        if (RefActor == nullptr)
        {
            return WorldTransf;
        }
        return GetRelativeTransform(RefActor->GetTransform(), WorldTransf);
    }

    /**
     * @brief Get the transform in reference frame. If RefActor==nullptr, return WorldTransf
     *
     * @param RefActor
     * @param WorldTransf Transform in world frame
     * @return FTransform Transform in reference frame
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetRelativeTransformFromActor(const AActor* RefActor, const FTransform& WorldTransf)
    {
        return GetRelativeTransform(RefActor, WorldTransf);
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
     * @param RelativeTransf Transform in reference frame
     * @return FTransform Transform in world frame
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetWorldTransform(const FTransform& RefTransf, const FTransform& RelativeTransf)
    {
        FTransform worldTransf;

        FTransform::Multiply(&worldTransf, &RelativeTransf, &RefTransf);

        worldTransf.NormalizeRotation();

        return worldTransf;
    }

    /**
     * @brief Get the transform in world frame. If RefActor==nullptr, return RelativeTransf
     *
     * @param RefActor
     * @param RelativeTransf Transform in reference frame
     * @return FTransform Transform in world frame
     */
    static FTransform GetWorldTransform(const AActor* RefActor, const FTransform& RelativeTransf)
    {
        if (RefActor == nullptr)
        {
            return RelativeTransf;
        }
        return GetWorldTransform(RefActor->GetTransform(), RelativeTransf);
    }

    /**
     * @brief Get the transform in world frame. If RefActor==nullptr, return RelativeTransf
     *
     * @param RefActor
     * @param RelativeTransf Transform in reference frame
     * @return FTransform Transform in world frame
     */
    UFUNCTION(BlueprintCallable)
    static FTransform GetWorldTransformFromActor(const AActor* RefActor, const FTransform& RelativeTransf)
    {
        return GetWorldTransform(RefActor, RelativeTransf);
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
};
