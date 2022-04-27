// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#pragma once

// Unreal
#include "Components/MeshComponent.h"
#include "Engine/PostProcessVolume.h"
#include "Kismet/BlueprintFunctionLibrary.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRThreadUtils.h"

#include "RRUObjectUtils.generated.h"

class URRStaticMeshComponent;

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRUObjectUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    // UOBJECT GENERAL UTILS --
    //
    template<typename T>
    FORCEINLINE static T* CreateSelfSubobject(UObject* InOuter, const FString& InObjectUniqueName)
    {
        return Cast<T>(CreateSelfSubobject(InOuter, T::StaticClass(), InObjectUniqueName));
    }

    FORCEINLINE static UObject* CreateSelfSubobject(UObject* InOuter, UClass* InObjectClass, const FString& InObjectUniqueName)
    {
        if (URRThreadUtils::IsInsideConstructor())
        {
            return FObjectInitializer::Get().CreateDefaultSubobject(
                InOuter, FName(*InObjectUniqueName), InObjectClass, InObjectClass, true, false);
        }
        else
        {
            return InOuter
                     ? NewObject<UObject>(InOuter, InObjectClass, FName(*InObjectUniqueName))
                     : NewObject<UObject>(static_cast<UObject*>(GetTransientPackage()), InObjectClass, FName(*InObjectUniqueName));
        }
    }

    template<typename T>
    FORCEINLINE static bool HasSubobject(UObject* InParentObj, const FString& InChildName)
    {
        TArray<UObject*> defaultSubobjects;
        InParentObj->GetDefaultSubobjects(defaultSubobjects);

        int32 index = -1;
        while (defaultSubobjects.FindItemByClass<T>(nullptr, &index))
        {
            if (defaultSubobjects[index]->GetName() == InChildName)
            {
                UE_LOG(LogTemp, Display, TEXT("FOUND SUBOBJECT %d - %s"), index, *InChildName);
                break;
            }
            else
            {
                defaultSubobjects.RemoveAt(index);
                index = -1;
            }
        }

        return (index >= 0);
    }

    FORCEINLINE static FString ComposeDynamicResourceName(const FString& InPrefix, const FString& InResourceUniqueName)
    {
        return FString::Printf(TEXT("%s%s"), *InPrefix, *InResourceUniqueName);
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // COMPONENT UTILS --
    //
    UFUNCTION()
    static void SetupComponentTick(UActorComponent* InComponent, bool bIsTickEnabled);
    UFUNCTION()
    static void SetupDefaultRootComponent(AActor* InActor);

    template<typename T, typename = TEnableIf<TIsDerivedFrom<T, UPrimitiveComponent>::Value>>
    static void ConfigureComponentPhysics(T* InComponent,
                                          bool bIsPhysicsEnabled,
                                          bool bIsCollisionEnabled,
                                          bool bIsOverlapEventEnabled)
    {
        // Setup [childComp] Physics & Collisions if it is a primitive component
        auto* primComp = CastChecked<UPrimitiveComponent>(InComponent);

        // Collision
        primComp->SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);
        if (bIsCollisionEnabled)
        {
            primComp->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
            primComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
            primComp->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
            primComp->SetNotifyRigidBodyCollision(true);
        }
        else if (bIsOverlapEventEnabled)
        {
            primComp->SetCollisionProfileName(TEXT("OverlapAllDynamic"));
            primComp->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
            primComp->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
        }
        else
        {
            primComp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
            primComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            primComp->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
        }

        // Overlap Event
        primComp->SetGenerateOverlapEvents(bIsOverlapEventEnabled);

        // Physics (last)
        primComp->SetSimulatePhysics(bIsPhysicsEnabled);
    }

    static void AttachComponentToComponent(
        USceneComponent* InChildComp,
        USceneComponent* InParentComp,
        const FTransform& InRelativeTransf = FTransform::Identity,
        const FAttachmentTransformRules& InAttachmentRules = FAttachmentTransformRules::KeepRelativeTransform,
        const TCHAR* InSocketName = nullptr);

    template<typename T>
    static T* CreateChildComponent(AActor* InActor,
                                   const FString& InUniqueName,
                                   bool bIsPhysicsEnabled = false,
                                   bool bIsCollisionEnabled = false,
                                   bool bIsOverlapEventEnabled = false)
    {
        T* newChildComp = Cast<T>(CreateSelfSubobject(InActor, T::StaticClass(), InUniqueName));
        verify(newChildComp);
        RegisterActorComponent(newChildComp);

        // Let [childComp] be owned by [InActor] to be memory-managed by GC.
        // That will contribute to [InActor]'s GetOverlappingActors check!
        if (InActor)
        {
            InActor->AddOwnedComponent(newChildComp);
        }

        if (Cast<UPrimitiveComponent>(newChildComp))
        {
            ConfigureComponentPhysics(newChildComp, bIsPhysicsEnabled, bIsCollisionEnabled, bIsOverlapEventEnabled);
        }
        return newChildComp;
    }

    template<typename T>
    static T* CreateAndAttachChildComponent(AActor* InActor,
                                            const FString& InUniqueName,
                                            const FTransform& InRelativeTransf = FTransform::Identity,
                                            USceneComponent* InParentComp = nullptr,
                                            const TCHAR* InAttachSocketName = nullptr)
    {
        T* newChildComp = Cast<T>(CreateSelfSubobject(InActor, T::StaticClass(), InUniqueName));
        verify(newChildComp);
        RegisterActorComponent(newChildComp);

        // (NOTE) Attachment setup must be done before SetSimulatePhysics()
        AttachComponentToComponent(newChildComp,
                                   InParentComp ? InParentComp : InActor->GetRootComponent(),
                                   InRelativeTransf,
                                   FAttachmentTransformRules::KeepRelativeTransform,
                                   InAttachSocketName);

        // Let [childComp] be owned by [InActor] to be memory-managed by GC.
        // That will also contribute to [InActor]'s GetOverlappingActors check!
        if (InActor)
        {
            InActor->AddOwnedComponent(newChildComp);
#if WITH_EDITOR
            // !! IMPORTANT FOR GET THESE DISPLAYED UNDER COMPONENTS TREE VIEW
            InActor->AddInstanceComponent(newChildComp);
#endif
        }
        return newChildComp;
    }

    template<typename TMeshComponent>
    FORCEINLINE static TMeshComponent* CreateMeshComponent(AActor* InActor,
                                                           const FString& InObjMeshUniqueName,
                                                           const FString& InMeshCompUniqueName,
                                                           const FTransform& InRelativeTransf,
                                                           bool bInIsOwningActorStationary,
                                                           bool bInIsPhysicsEnabled,
                                                           bool bInIsCollisionEnabled,
                                                           USceneComponent* InParentComp = nullptr)
    {
        // 1 - Create --
        // Also already adding to InActor's OwnedComponents here-in!
        TMeshComponent* meshComp =
            CreateAndAttachChildComponent<TMeshComponent>(InActor, InMeshCompUniqueName, InRelativeTransf, InParentComp);
        meshComp->MeshUniqueName = InObjMeshUniqueName;

#if RAPYUTA_SIM_VISUAL_DEBUG
        UE_LOG(LogTemp,
               Display,
               TEXT("[%s] MESH COMP [%s] ATTACHED TO %s|%s"),
               *InActor->GetName(),
               *meshComp->GetName(),
               *meshComp->GetAttachParent()->GetName(),
               *InRelativeTransf.ToString());
#endif

        // 2 - Initialize --
        // Collision --
        // Not physics-enabled does not necessarily mean No-collision, like query-only collision.
        meshComp->SetCollisionModeAvailable(bInIsCollisionEnabled);

        // Overlapping --
        if (!bInIsPhysicsEnabled && !bInIsCollisionEnabled)
        {
            meshComp->EnableOverlapping();
        }

        // Stationary --
        meshComp->bIsStationary = bInIsOwningActorStationary;
        if (bInIsOwningActorStationary)
        {
            meshComp->SetMobility(EComponentMobility::Stationary);
            meshComp->SetCollisionObjectType(ECollisionChannel::ECC_WorldStatic);
            meshComp->SetEnableGravity(false);
#if RAPYUTA_SIM_DEBUG
            meshComp->LockSelf();
#endif
        }
        else
        {
            meshComp->SetMobility(EComponentMobility::Movable);
        }

        // Static mesh (if applicable) --
        if constexpr (TIsDerivedFrom<TMeshComponent, URRStaticMeshComponent>::IsDerived)
        {
            // Mesh -- Must be after above configuration, based on which particular mesh properties are verified!
            // Material instance created here-in, thus no need to create a default one like based on [CMAT_NAME_GENERIC]
            meshComp->SetMesh(URRGameSingleton::Get()->GetStaticMesh(InObjMeshUniqueName));
        }
        // else Runtime mesh, if available, would be fetched on-the-fly later
        // (due to their changeable content (by users either offline or online), thus Sim would never storing ones on disk)

        // Physics (last)--
        const bool bIsStationary = InObjMeshUniqueName.Equals(URRGameSingleton::SHAPE_NAME_PLANE);
        // If using Custom Physics Engine, also create [PhysicsComp] here-in!
        // (NOTE) This will DETACH meshComp from its Parent in case of [bInIsPhysicsEnabled]
        meshComp->Initialize(bIsStationary, bInIsPhysicsEnabled);

        return meshComp;
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // ACTOR UTILS --
    //
    UFUNCTION()
    static void SetupActorTick(AActor* InActor, bool bIsTickEnabled, float InTickInterval = 0.f);
    UFUNCTION()
    static void RegisterActorComponent(UActorComponent* Comp);

    // [FIND ACTOR BY NAME/SUBNAME] --
    // GetAllActors() is expensive
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
        UE_LOG(LogTemp, Log, TEXT("Actor named [%s] is unavailable."), *InName);
        return nullptr;
    }

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
        UE_LOG(LogTemp, Log, TEXT("Actor name containing [%s] is unavailable."), *InSubname);
        return nullptr;
    }

    UFUNCTION()
    static AActor* FindEnvironmentActor(UWorld* InWorld)
    {
        // There is only one common [Environment] actor of all Scene instances!
        return FindActorBySubname<AActor>(InWorld, TEXT("RapyutaEnvironment"));
    }

    UFUNCTION()
    static AActor* FindSkyActor(UWorld* InWorld)
    {
        // There is only one common [Sky] actor of all Scene instances!
        return FindActorBySubname<AActor>(InWorld, TEXT("RapyutaSky"));
    }

    UFUNCTION()
    static ASkyLight* FindSkyLight(UWorld* InWorld)
    {
        // There is only one common [SkyLight] actor of all Scene instances!
        return FindActorBySubname<ASkyLight>(InWorld, TEXT("SkyLight"));
    }

    UFUNCTION()
    static APostProcessVolume* FindPostProcessVolume(UWorld* InWorld)
    {
        // There is only one common [PostProcessVolume] actor of all Scene instances!
        return FindActorBySubname<APostProcessVolume>(InWorld, TEXT("PostProcessVolume"));
    }

    // TActorSpawnInfo: [FRRActorSpawnInfo], etc.
    template<typename T, typename TActorSpawnInfo>
    static T* SpawnSimActor(
        UWorld* InWorld,
        int8 InSceneInstanceId,
        const TActorSpawnInfo& InActorSpawnInfo,
        const ESpawnActorCollisionHandlingMethod CollisionHandlingType = ESpawnActorCollisionHandlingMethod::AlwaysSpawn)
    {
        if (false == InActorSpawnInfo.IsValid(true))
        {
            return nullptr;
        }

        // [TypeClass] has more priority than [T::StaticClass()]
        // In case [TypeClass] is NULL => T must be an [AActor] or its child class
        if ((InActorSpawnInfo.TypeClass == nullptr) && (T::StaticClass() != AActor::StaticClass()) &&
            (false == T::StaticClass()->IsChildOf(AActor::StaticClass())))
        {
            UE_LOG(LogTemp, Fatal, TEXT("[SpawnSimActor]: NULL SPAWN TYPE-CLASS && A NON-AACTOR CLASS!"));
            return nullptr;
        }

        // [SIM ACTOR] --
        //
        T::SSceneInstanceId = InSceneInstanceId;
        //
        FActorSpawnParameters spawnInfo;
        spawnInfo.Name = FName(*InActorSpawnInfo.UniqueName);
        spawnInfo.SpawnCollisionHandlingOverride = CollisionHandlingType;
        T* newSimActor = InWorld->SpawnActor<T>(
            InActorSpawnInfo.TypeClass ? static_cast<UClass*>(InActorSpawnInfo.TypeClass) : static_cast<UClass*>(T::StaticClass()),
            InActorSpawnInfo.ActorTransform,
            spawnInfo);

        if (newSimActor)
        {
            // http://klamp.works/2015/10/09/call-template-method-of-template-class-from-template-function.html
            newSimActor->template InitializeWithSpawnInfo<TActorSpawnInfo>(InActorSpawnInfo);

#if WITH_EDITOR
            // [newObjectActor] ActorUniqueName is set inside the CTOR
            // In Editor, Use the id itself for actor's label as well, just for sake of verification.
            newSimActor->SetActorLabel(InActorSpawnInfo.UniqueName);
#endif

#if RAPYUTA_SIM_DEBUG
            UE_LOG(LogTemp,
                   Warning,
                   TEXT("[%s:%d] SIM ACTOR SPAWNED: [%s] => [%s]\nat %s -> %s"),
                   *InActorSpawnInfo.UniqueName,
                   newSimActor,
                   *spawnInfo.Name.ToString(),
                   *newSimActor->GetName(),
                   *InActorSpawnInfo.ActorTransform.ToString(),
                   *newSimActor->GetActorTransform().ToString());
#endif
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("[%s] FAILED SPAWNING OBJECT ACTOR!!!"), *InActorSpawnInfo.UniqueName);
            return nullptr;
        }

        return newSimActor;
    }

    static ARRBaseActor* SpawnSimActor(
        UWorld* InWorld,
        int8 InSceneInstanceId,
        UClass* InActorClass,
        const FString& InEntityModelName,
        const FString& InActorName = EMPTY_STR,
        const FTransform& InActorTransform = FTransform::Identity,
        const ESpawnActorCollisionHandlingMethod InCollisionHandlingType = ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

    template<typename T>
    static FVector GetActorGroupCenter(const TArray<T*>& InActors)
    {
        FVector sumLocation = FVector::ZeroVector;
        for (const auto& actor : InActors)
        {
            sumLocation += actor->GetActorLocation();
        }
        return sumLocation / InActors.Num();
    }

    template<typename T>
    static void HuddleActors(const TArray<T*>& InActors)
    {
        // (NOTE) Actors should be not touching the floor so they could sweep
        const FVector center = URRUObjectUtils::GetActorGroupCenter(InActors);
        const auto actorsNum = InActors.Num();
        const auto actorsNumHalf = FMath::CeilToInt(0.5f * actorsNum);
        for (auto i = actorsNumHalf; i < actorsNum; ++i)
        {
            auto& actor = InActors[i];
            actor->AddActorWorldOffset(center - actor->GetActorLocation(), true);
        }
        for (auto i = actorsNumHalf - 1; i >= 0; --i)
        {
            auto& actor = InActors[i];
            actor->AddActorWorldOffset(center - actor->GetActorLocation(), true);
        }
    }

    static UMaterialInstanceDynamic* CreateMeshCompMaterialInstance(UMeshComponent* InMeshComp,
                                                                    int32 InMaterialIndex,
                                                                    const FString& InMaterialInterfaceName);
};
