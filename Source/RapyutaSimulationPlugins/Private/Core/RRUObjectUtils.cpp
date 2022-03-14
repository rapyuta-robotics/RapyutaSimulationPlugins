#include "Core/RRUObjectUtils.h"

// UE
#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Core/RRMathUtils.h"

void URRUObjectUtils::SetupActorTick(AActor* InActor, bool bIsTickEnabled, float InTickInterval)
{
    // Tick if not required could be disabled to improve performance
    InActor->PrimaryActorTick.bCanEverTick = bIsTickEnabled;
    InActor->PrimaryActorTick.bStartWithTickEnabled = bIsTickEnabled;
    InActor->SetActorTickEnabled(bIsTickEnabled);
    if (bIsTickEnabled)
    {
        InActor->PrimaryActorTick.Target = InActor;
    }
    if (InTickInterval > 0.f)
    {
        InActor->PrimaryActorTick.TickInterval = InTickInterval;
    }

#if RAPYUTA_SIM_DEBUG
    // These are for troubleshooting only, in case Tick does not work even having been enabled.
    // These also don't apply the same for all actors
    InActor->PrimaryActorTick.bAllowTickOnDedicatedServer = bIsTickEnabled;
    InActor->PrimaryActorTick.bHighPriority = bIsTickEnabled;
    InActor->PrimaryActorTick.bRunOnAnyThread = bIsTickEnabled;
    InActor->PrimaryActorTick.RegisterTickFunction(InActor->GetLevel());
    // --> Call Child component's RegisterAllComponentTickFunctions(bRegister);
    InActor->RegisterAllActorTickFunctions(bIsTickEnabled, bIsTickEnabled);
#endif
}

void URRUObjectUtils::SetupComponentTick(UActorComponent* InComponent, bool bIsTickEnabled)
{
    InComponent->PrimaryComponentTick.bCanEverTick = bIsTickEnabled;
    InComponent->PrimaryComponentTick.bStartWithTickEnabled = bIsTickEnabled;
    InComponent->bTickInEditor = bIsTickEnabled;
}

void URRUObjectUtils::SetupDefaultRootComponent(AActor* InActor)
{
    if (InActor->GetRootComponent())
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] RootComponent has already been set up!"), *InActor->GetName());
        return;
    }

    USceneComponent* defaultRoot = CreateSelfSubobject<USceneComponent>(InActor, TEXT("DefaultRoot"));

    if (!(defaultRoot && InActor->SetRootComponent(defaultRoot)))
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("[%s] Failed setting up root component!"), *InActor->GetName());
        return;
    }
#if WITH_EDITOR
    defaultRoot->bVisualizeComponent = false;
#endif
}

void URRUObjectUtils::RegisterActorComponent(UActorComponent* InComp)
{
    // This must not inside ctor, which would cause crash!
    if (false == URRThreadUtils::IsInsideConstructor())
    {
        // This also helps the component to be visibile under InActor's Details Panel in the Editor
        InComp->RegisterComponent();
    }
}

void URRUObjectUtils::AttachComponentToComponent(USceneComponent* InChildComp,
                                                 USceneComponent* InParentComp,
                                                 const FTransform& InRelativeTransf,
                                                 const FAttachmentTransformRules& InAttachmentRules,
                                                 const TCHAR* InSocketName)
{
    // https://forums.unrealengine.com/t/attach-actor-to-component-doesnt-work-if-physics-has-ever-been-enabled/9774
    // It has been observed that if InChildComp has Physics simulation enabled,
    // the attachment will be done to the owner's root comp.
    if (URRThreadUtils::IsInsideConstructor())
    {
        // UE4: AttachToComponent when called from a ctor is only setup the attachment
        // and will always be treated as KeepRelative
        InChildComp->SetupAttachment(InParentComp, InSocketName);
    }
    else
    {
        // UE4: Attach this component to another scene component, optionally at a named socket.
        // It is valid to call this on components whether or not they have been Registered,
        // However, from ctor or if not having been registered it is preferable to use SetupAttachment.
        InChildComp->AttachToComponent(InParentComp, InAttachmentRules, InSocketName);
    }
    InChildComp->SetRelativeTransform(InRelativeTransf);

    // Verification
    if (IsValid(InParentComp))
    {
        if (InChildComp->GetAttachParent() != InParentComp)
        {
            UE_LOG(LogRapyutaCore,
                   Fatal,
                   TEXT("%s's AttachParent (%s) # (%s) Parent"),
                   *InChildComp->GetName(),
                   *InChildComp->GetAttachParent()->GetName(),
                   *InParentComp->GetName());
        }
    }
}

ARRBaseActor* URRUObjectUtils::SpawnSimActor(UWorld* InWorld,
                                             int8 InSceneInstanceId,
                                             UClass* InActorClass,
                                             const FString& InActorName,
                                             const FTransform& InActorTransform,
                                             const ESpawnActorCollisionHandlingMethod InCollisionHandlingType)
{
    // This is needed for any actor that is spawned after Sim initialization, when its BeginPlay() is invoked later
    ARRBaseActor::SSceneInstanceId = InSceneInstanceId;

    FActorSpawnParameters spawnInfo;
    // (dnote) To be checked again, not know why this naming does not work??
    spawnInfo.Name = InActorName.IsEmpty() ? FName(*FString::Printf(TEXT("%d_%s"), InSceneInstanceId, *InActorClass->GetName()))
                                           : FName(*InActorName);
    spawnInfo.SpawnCollisionHandlingOverride = InCollisionHandlingType;
    ARRBaseActor* newActor = InWorld->SpawnActor<ARRBaseActor>(InActorClass, InActorTransform, spawnInfo);
    verify(newActor);
    // This is set in ctor
    verify(newActor->SceneInstanceId == InSceneInstanceId);

#if WITH_EDITOR
    // [newObjectActor] ActorUniqueName is set inside the CTOR
    // In Editor, Use the id itself for actor's label as well, just for sake of verification.
    newActor->SetActorLabel(spawnInfo.Name.ToString());
#endif

    // Initializing itself
    verify(newActor->Initialize());

    return newActor;
}

void URRUObjectUtils::SetCameraToLookAt(UCameraComponent* InCameraComponent, const FVector& InTargetLocation)
{
    // Reference: UnrealEngine/Engine/Source/Runtime/UMG/Public/Components/Viewport.h - SetLookAtLocation()
    FMatrix newCameraViewMatrix =
        URRMathUtils::ComputeViewMatrixFromObjectToTarget(InCameraComponent->GetComponentTransform(), InTargetLocation);

    InCameraComponent->SetWorldTransform(FTransform(newCameraViewMatrix.Rotator(), newCameraViewMatrix.GetOrigin()));
}
