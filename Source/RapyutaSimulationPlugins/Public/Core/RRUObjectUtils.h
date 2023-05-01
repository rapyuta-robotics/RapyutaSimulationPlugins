/**
 * @file RRUObjectUtils.h
 * @brief UObject general utils
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// Unreal
#include "Components/MeshComponent.h"
#include "Engine/PostProcessVolume.h"
#include "Kismet/BlueprintFunctionLibrary.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRGameState.h"
#include "Core/RRTextureData.h"
#include "Core/RRThreadUtils.h"

#include "RRUObjectUtils.generated.h"

class ARRBaseActor;
class ARRMeshActor;
class URRStaticMeshComponent;

/**
 * @brief UObject general utils.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRUObjectUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    template<typename T>
    FORCEINLINE static T* CreateSelfSubobject(UObject* InOuter, const FString& InObjectUniqueName)
    {
        return Cast<T>(CreateSelfSubobject(InOuter, T::StaticClass(), InObjectUniqueName));
    }

    /**
     * @brief Use CreateDefaultSubobject or NewObject based on where this method is called.
     * Uses #URRThreadUtils::IsInsideConstructor.
     * @param InOuter test
     * @param InObjectClass
     * @param InObjectUniqueName
     * @return static UObject*
     *
     * @sa[CreateDefaultSubobject](https://docs.unrealengine.com/5.1/en-US/API/Runtime/CoreUObject/UObject/UObject/CreateDefaultSubobject/2/)
     * @sa[NewObject](https://docs.unrealengine.com/5.1/en-US/ProgrammingAndScripting/ProgrammingWithCPP/UnrealArchitecture/Objects/Creation/)
     */
    FORCEINLINE static UObject* CreateSelfSubobject(UObject* InOuter, UClass* InObjectClass, const FString& InObjectUniqueName)
    {
        if (URRThreadUtils::IsInsideConstructor())
        {
            return FObjectInitializer::Get().CreateDefaultSubobject(
                InOuter, FName(*InObjectUniqueName), InObjectClass, InObjectClass, true, false);
        }
        else
        {
            return InOuter ? NewObject<UObject>(InOuter, InObjectClass, FName(*InObjectUniqueName))
                           : NewObject<UObject>(
                                 static_cast<UObject*>(GetTransientPackage()), InObjectClass, FName(*InObjectUniqueName));
        }
    }

    /**
     * @brief Return true if InParentObj has subobjects with InChildName.
     *
     * @tparam T
     * @param InParentObj
     * @param InChildName
     * @return static bool
     */
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
                UE_LOG_WITH_INFO(LogTemp, Display, TEXT("FOUND SUBOBJECT %d - %s"), index, *InChildName);
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
    static USceneComponent* SetupDefaultRootComponent(AActor* InActor);

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

    static void DisableNavImpactAndPhysicsCollision(UPrimitiveComponent* InComponent)
    {
        InComponent->SetCanEverAffectNavigation(false);
        InComponent->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
        InComponent->SetSimulatePhysics(false);
    }

    /**
     * @brief Use SetupAttachment or AttachToComponent based on where this method is called.
     * Uses #URRThreadUtils::IsInsideConstructor.
     *
     * @param InChildComp
     * @param InParentComp
     * @param InRelativeTransf
     * @param InAttachmentRules
     * @param InSocketName
     *
     * @sa[SetupAttachment](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Components/USceneComponent/SetupAttachment/)
     * @sa[AttachToComponent](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Components/USceneComponent/AttachToComponent/)
     */
    static void AttachComponentToComponent(
        USceneComponent* InChildComp,
        USceneComponent* InParentComp,
        const FTransform& InRelativeTransf = FTransform::Identity,
        const FAttachmentTransformRules& InAttachmentRules = FAttachmentTransformRules::KeepRelativeTransform,
        const TCHAR* InSocketName = nullptr);

    /**
     * @brief Create a Child Component object and attach to InActor and calls #ConfigureComponentPhysics.
     *
     * @tparam T
     * @param InActor
     * @param InUniqueName
     * @param bIsPhysicsEnabled
     * @param bIsCollisionEnabled
     * @param bIsOverlapEventEnabled
     * @return T*
     */
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

    /**
     * @brief Create a Mesh Component, attach to InActor, set parameters.
     *
     * @tparam TMeshComponent
     * @param InActor
     * @param InObjMeshUniqueName
     * @param InMeshCompUniqueName
     * @param InRelativeTransf
     * @param bInIsOwningActorStationary
     * @param bInIsPhysicsEnabled
     * @param bInIsCollisionEnabled
     * @param InParentComp
     * @return static TMeshComponent*
     */
    template<typename TMeshComponent>
    FORCEINLINE static TMeshComponent* CreateMeshComponent(AActor* InActor,
                                                           const FString& InObjMeshUniqueName,
                                                           const FString& InMeshCompUniqueName,
                                                           const FTransform& InRelativeTransf,
                                                           bool bInIsOwningActorStationary,
                                                           bool bInIsPhysicsEnabled,
                                                           bool bInIsCollisionEnabled,
                                                           bool bInIsOverlapEventEnabled = false,
                                                           USceneComponent* InParentComp = nullptr)
    {
        // 1 - Create --
        // Also already adding to InActor's OwnedComponents here-in!
        TMeshComponent* meshComp =
            CreateAndAttachChildComponent<TMeshComponent>(InActor, InMeshCompUniqueName, InRelativeTransf, InParentComp);
        meshComp->MeshUniqueName = InObjMeshUniqueName;

#if RAPYUTA_SIM_VISUAL_DEBUG
        UE_LOG_WITH_INFO(LogTemp,
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
            meshComp->EnableOverlapping(bInIsOverlapEventEnabled);
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

    /**
     * @brief Uses URRThreadUtils::IsInsideConstructor() to avoid crash by calling RegisterComponents() outside of constructor.
     *
     */
    UFUNCTION()
    static void RegisterActorComponent(UActorComponent* Comp);

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
        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("Actor named [%s] is unavailable."), *InName);
        return nullptr;
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
        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("Actor name containing [%s] is unavailable."), *InSubname);
        return nullptr;
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
        }
        return actors;
    }

    UFUNCTION()
    static AActor* FindEnvironmentActor(UWorld* InWorld)
    {
        // There is only one common [Environment] actor of all Scene instances!
        return FindActorBySubname<AActor>(InWorld, TEXT("MainEnvironment"));
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
    static AActor* FindFloorActor(UWorld* InWorld)
    {
        return FindActorBySubname<AActor>(InWorld, TEXT("MainFloor"));
    }

    UFUNCTION()
    static AActor* FindWallActor(UWorld* InWorld)
    {
        return FindActorBySubname<AActor>(InWorld, TEXT("MainWall"));
    }

    UFUNCTION()
    static TArray<AStaticMeshActor*> FindStaticMeshActorListByMeshSubname(
        UWorld* InWorld,
        const FString& InMeshSubname,
        bool bCreateMaterialInstanceDynamic,
        const ESearchCase::Type InCaseType = ESearchCase::IgnoreCase)
    {
        TArray<AStaticMeshActor*> actors;
        for (TActorIterator<AStaticMeshActor> actorItr(InWorld); actorItr; ++actorItr)
        {
            auto* meshComp = actorItr->GetStaticMeshComponent();
            if (meshComp->GetStaticMesh()->GetName().Contains(InMeshSubname, InCaseType))
            {
                if (bCreateMaterialInstanceDynamic)
                {
                    for (auto i = 0; i < meshComp->GetMaterials().Num(); ++i)
                    {
                        meshComp->CreateDynamicMaterialInstance(i, meshComp->GetMaterial(i));
                    }
                }
                actors.Add(*actorItr);
            }
        }
        return actors;
    }

    template<typename T>
    static TArray<T*> FindActorListByType(UWorld* InWorld)
    {
        TArray<T*> actors;
        for (TActorIterator<T> actorItr(InWorld); actorItr; ++actorItr)
        {
            actors.Add(*actorItr);
        }
        return actors;
    }

    UFUNCTION()
    static APostProcessVolume* FindPostProcessVolume(UWorld* InWorld)
    {
        // There is only one common [PostProcessVolume] actor of all Scene instances!
        return FindActorBySubname<APostProcessVolume>(InWorld, TEXT("PostProcessVolume"));
    }

    // TActorSpawnInfo: [FRRActorSpawnInfo], etc.
    /**
     * @brief Spawn a generic actor that is either mesh-based or mesh-free & initialize it with actor spawn info
     * @tparam T
     * @tparam TActorSpawnInfo
     * @param InWorld
     * @param InSceneInstanceId
     * @param InActorSpawnInfo
     * @param CollisionHandlingType
     * @return T*
     */
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
            UE_LOG_WITH_INFO(LogTemp, Fatal, TEXT("NULL SPAWN TYPE-CLASS && A NON-AACTOR CLASS!"));
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

#if RAPYUTA_SIM_VERBOSE
            UE_LOG_WITH_INFO(LogTemp,
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
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("[%s] FAILED SPAWNING OBJECT ACTOR!!!"), *InActorSpawnInfo.UniqueName);
            return nullptr;
        }

        return newSimActor;
    }

    /**
     * @brief Spawn a generic actor that is either mesh-based or mesh-free
     *
     * @param InWorld
     * @param InSceneInstanceId
     * @param InActorClass
     * @param InActorName
     * @param InActorTransform
     * @param InCollisionHandlingType
     * @return ARRBaseActor*
     */
    static ARRBaseActor* SpawnSimActor(
        UWorld* InWorld,
        int8 InSceneInstanceId,
        UClass* InActorClass,
        const FString& InEntityModelName,
        const FString& InActorName = EMPTY_STR,
        const FTransform& InActorTransform = FTransform::Identity,
        const ESpawnActorCollisionHandlingMethod InCollisionHandlingType = ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

    FORCEINLINE static FVector GetRelativeLocFrom(const AActor* InActor, const AActor* InBaseActor)
    {
        return InBaseActor->GetTransform().InverseTransformPosition(InActor->GetActorLocation());
    }

    FORCEINLINE static FQuat GetRelativeQuatFrom(const FQuat& InQuat, const AActor* InBaseActor)
    {
        return InBaseActor->GetTransform().InverseTransformRotation(InQuat);
    }

    FORCEINLINE static FQuat GetRelativeQuatFrom(const AActor* InActor, const AActor* InBaseActor)
    {
        return GetRelativeQuatFrom(InActor->GetActorQuat(), InBaseActor);
    }

    FORCEINLINE static FRotator GetRelativeRotFrom(const AActor* InActor, const AActor* InBaseActor)
    {
        return GetRelativeQuatFrom(InActor, InBaseActor).Rotator();
    }

    FORCEINLINE static FVector GetDirectedExtent(const FVector& InNormal, const FVector& InExtent)
    {
        // Normal coord -> Extent coord
        return FVector((0.f == InNormal.X) ? -InExtent.X : InExtent.X,
                       (0.f == InNormal.Y) ? -InExtent.Y : InExtent.Y,
                       (0.f == InNormal.Z) ? -InExtent.Z : InExtent.Z);
    }

    static void GetActorCenterAndBoundingBoxVertices(const AActor* InActor,
                                                     const AActor* InBaseActor,
                                                     TArray<FVector>* OutCenterAndVertices3D,
                                                     bool bInIncludeNonColliding = true);

    template<typename TActor>
    static void GetHomoActorGroupCenterAndBoundingBoxVertices(const TArray<TActor*>& InActors,
                                                              const AActor* InBaseActor,
                                                              TArray<FVector>* OutCenterAndVertices3D,
                                                              bool bInIncludeNonColliding = true)
    {
        OutCenterAndVertices3D->Reset();
        ARRGameState* gameState = URRCoreUtils::GetGameState<ARRGameState>(InActors[0]);

        // [InActors]' World bounding box
        FBox groupWorldBox(ForceInit);
        for (const auto& actor : InActors)
        {
            groupWorldBox += actor->GetComponentsBoundingBox(bInIncludeNonColliding);
        }
        FVector centerWorld, extentsWorld;
        groupWorldBox.GetCenterAndExtents(centerWorld, extentsWorld);

        // [InActors]'s group bounding box with rotation (in BaseActor frame or World frame)
        if (InBaseActor)
        {
            // Calculate [centerBase] to [InBaseActor] frame
            // UnrealEngine/Engine/Plugins/2D/Paper2D/Source/Paper2D/Private/PaperSpriteComponent.cpp:179
            const FTransform homoGroupTransform = GetHomoActorGroupTransform(InActors);
            const FTransform baseTransform = homoGroupTransform.GetRelativeTransform(InBaseActor->GetTransform());
            const FVector centerBase = baseTransform.GetTranslation();

            // [0]: centerBase
            OutCenterAndVertices3D->Emplace(centerBase);

            // [1-8]: verticesBase
            // Calculate global vertices as : [centerBase] + Rotated [extentsWorld]
            for (auto i = 0; i < gameState->ENTITY_BOUNDING_BOX_VERTEX_NORMALS.Num(); ++i)
            {
                OutCenterAndVertices3D->Emplace(centerBase + baseTransform.GetRotation().RotateVector(GetDirectedExtent(
                                                                 gameState->GetEntityBBVertexNormal(i), extentsWorld)));
            }
        }
        else
        {
            // [0]: centerWorld
            OutCenterAndVertices3D->Emplace(centerWorld);

            // [1-8]: verticesWorld
            // Calculate global vertices as : [centerWorld] + Rotated [extentsWorld]
            for (auto i = 0; i < gameState->ENTITY_BOUNDING_BOX_VERTEX_NORMALS.Num(); ++i)
            {
                OutCenterAndVertices3D->Emplace(centerWorld +
                                                GetDirectedExtent(gameState->GetEntityBBVertexNormal(i), extentsWorld));
            }
        }
    }

    static FVector GetActorExtent(AActor* InActor, bool bOnlyCollidingComponents = true, bool bIncludeFromChildActors = false)
    {
        FVector actorOrigin, actorExtent;
        InActor->GetActorBounds(bOnlyCollidingComponents, actorOrigin, actorExtent, bIncludeFromChildActors);
        return actorExtent;
    }

    static FVector GetActorSize(AActor* InActor, bool bOnlyCollidingComponents = true, bool bIncludeFromChildActors = false)
    {
        return 2.f * GetActorExtent(InActor, bOnlyCollidingComponents, bIncludeFromChildActors);
    }

    static void DrawActorBoundingBox(AActor* InActor)
    {
        FVector actorCenter, actorExtent;
        InActor->GetActorBounds(false, actorCenter, actorExtent);
        DrawDebugBox(InActor->GetWorld(), actorCenter, actorExtent, FColor::Yellow, false, 2.f, 0, 2.f);
    }

    template<typename T>
    static FTransform GetHomoActorGroupTransform(const TArray<T*>& InActors)
    {
        return FTransform(InActors[0]->GetActorRotation(), GetActorGroupCenter(InActors));
    }

    template<typename T>
    static FVector GetActorGroupCenter(const TArray<T*>& InActors)
    {
        FVector sumLocation = FVector::ZeroVector;
        for (const auto& actor : InActors)
        {
            sumLocation += actor->GetActorLocation();
        }
        return (InActors.Num() > 0) ? sumLocation / InActors.Num() : FVector::ZeroVector;
    }

    template<typename T>
    static FVector GetActorGroupListCenter(const TArray<TArray<T*>>& InActorGroups)
    {
        FVector sumLocation = FVector::ZeroVector;
        for (const auto& actorGroup : InActorGroups)
        {
            sumLocation += GetActorGroupCenter(actorGroup);
        }
        return (InActorGroups.Num() > 0) ? sumLocation / InActorGroups.Num() : FVector::ZeroVector;
    }

    static bool IsActorOverlapping(AActor* InActor)
    {
        for (UActorComponent* ownedComp : InActor->GetComponents())
        {
            if (UPrimitiveComponent* primComp = Cast<UPrimitiveComponent>(ownedComp))
            {
                TSet<UPrimitiveComponent*> overlapSet;
                primComp->GetOverlappingComponents(overlapSet);
                if (overlapSet.Num() > 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

    static bool IsEnvStaticMeshActorsOverlapEventEnabled(UWorld* InWorld)
    {
        for (TActorIterator<AStaticMeshActor> actorItr(InWorld); actorItr; ++actorItr)
        {
            if (!actorItr->GetStaticMeshComponent()->GetGenerateOverlapEvents())
            {
                UE_LOG_WITH_INFO(LogTemp, Display, TEXT("%s GenerateOverlapEvents disabled"), *actorItr->GetName());
                return false;
            }
        }
        return true;
    }

    static void SetEnvStaticMeshActorsOverlapEventEnabled(UWorld* InWorld, bool bEnabled)
    {
        for (TActorIterator<AStaticMeshActor> actorItr(InWorld); actorItr; ++actorItr)
        {
            actorItr->GetStaticMeshComponent()->SetGenerateOverlapEvents(bEnabled);
        }
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
    static FString GetSegMaskDepthStencilsAsText(AActor* InActor);
    static bool GetPhysicsActorHandles(FBodyInstance* InBody1,
                                       FBodyInstance* InBody2,
                                       FPhysicsActorHandle& OutActorRef1,
                                       FPhysicsActorHandle& OutActorRef2);
    static void SetHomoLinearConstraintMotion(FConstraintInstance* InCI, const ELinearConstraintMotion InLinearMotion);
    static void SetHomoAngularConstraintMotion(FConstraintInstance* InCI, const EAngularConstraintMotion InAngularMotion);
    static UMaterialInstanceDynamic* CreateMeshCompMaterialInstance(UMeshComponent* InMeshComp,
                                                                    int32 InMaterialIndex,
                                                                    const FString& InMaterialInterfaceName);
    static int32 GetActorMaterialsNum(AActor* InActor);
    static UMaterialInstanceDynamic* GetActorBaseMaterial(AActor* InActor, int32 InMaterialIndex = 0);
    static bool ApplyMeshActorMaterialProps(AActor* InActor,
                                            const FRRMaterialProperty& InMaterialInfo,
                                            bool bApplyManufacturingAlbedo = true);
    static void ApplyMaterialProps(UMaterialInstanceDynamic* InMaterial,
                                   const FRRMaterialProperty& InMaterialInfo,
                                   bool bApplyManufacturingAlbedo = true);
    static bool SetMeshActorColor(AActor* InMeshActor, const FLinearColor& InColor);
    static void RandomizeActorAppearance(AActor* InActor, const FRRTextureData& InTextureData);
};
