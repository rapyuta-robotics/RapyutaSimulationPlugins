#include "Core/RRUObjectUtils.h"

// UE
#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMathUtils.h"
#include "Core/RRMeshActor.h"

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
                                             const FString& InEntityModelName,
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
    newActor->SetEntityModelName(InEntityModelName);

#if WITH_EDITOR
    // [newObjectActor] ActorUniqueName is set inside the CTOR
    // In Editor, Use the id itself for actor's label as well, just for sake of verification.
    newActor->SetActorLabel(spawnInfo.Name.ToString());
#endif

    // Initializing itself
    verify(newActor->Initialize());

    return newActor;
}
void URRUObjectUtils::GetActorCenterAndBoundingBoxVertices(const AActor* InActor,
                                                           TArray<FVector>& OutCenterAndVerticesWorld,
                                                           bool bInIncludeNonColliding)
{
    // [InActor]'s global Rotation
    const FQuat& actorQuat = InActor->GetActorQuat();

    // [InActor]'s Local bounding box
    FVector centerLocal, extentsLocal;
    FBox actorLocalBox = InActor->CalculateComponentsBoundingBoxInLocalSpace(bInIncludeNonColliding);
    actorLocalBox.GetCenterAndExtents(centerLocal, extentsLocal);

    // [InActor]'s global bounding box with rotation
    // Reference : DrawDebugBox()
    // (snote) The exact order of points is important. Consult with Robot Research team before adjusting.
    //
    // Transfom [centerLocal] to global
    const FVector& centerWorld = InActor->GetTransform().TransformPosition(centerLocal);
    OutCenterAndVerticesWorld.Emplace(centerWorld);

    // Calculate global vertices as : [centerWorld] + Rotated [ExtentsLocal]
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(extentsLocal.X, extentsLocal.Y, extentsLocal.Z)));
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(extentsLocal.X, -extentsLocal.Y, extentsLocal.Z)));
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(-extentsLocal.X, -extentsLocal.Y, extentsLocal.Z)));
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(-extentsLocal.X, extentsLocal.Y, extentsLocal.Z)));
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(extentsLocal.X, extentsLocal.Y, -extentsLocal.Z)));
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(extentsLocal.X, -extentsLocal.Y, -extentsLocal.Z)));
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(-extentsLocal.X, -extentsLocal.Y, -extentsLocal.Z)));
    OutCenterAndVerticesWorld.Emplace(centerWorld +
                                      actorQuat.RotateVector(FVector(-extentsLocal.X, extentsLocal.Y, -extentsLocal.Z)));

#if RAPYUTA_SIM_VISUAL_DEBUG
    if (IsInGameThread())
    {
        // Draw [InActor]'s bounding box before projecting it onto screen to get the 2D coordinates.
        OutCenterAndVerticesWorld.RemoveAt(0);
        URRFunctionLibrary::DrawActorBounds(InActor->GetWorld(), OutCenterAndVerticesWorld);
        OutCenterAndVerticesWorld.Insert(centerWorld, 0);
    }
#endif
}

// Ref: ConstraintInstance.cpp - GetActorRefs()
bool URRUObjectUtils::GetPhysicsActorHandles(FBodyInstance* InBody1,
                                             FBodyInstance* InBody2,
                                             FPhysicsActorHandle& OutActorRef1,
                                             FPhysicsActorHandle& OutActorRef2)
{
    FPhysicsActorHandle actorRef1 = InBody1 ? InBody1->ActorHandle : FPhysicsActorHandle();
    FPhysicsActorHandle actorRef2 = InBody2 ? InBody2->ActorHandle : FPhysicsActorHandle();
    // Do not create joint unless we have two actors or one of them is dynamic
    if ((!FPhysicsInterface::IsValid(actorRef1) || !FPhysicsInterface::IsRigidBody(actorRef1)) &&
        (!FPhysicsInterface::IsValid(actorRef2) || !FPhysicsInterface::IsRigidBody(actorRef2)))
    {
        return false;
    }
    if (FPhysicsInterface::IsValid(actorRef1) && FPhysicsInterface::IsValid(actorRef2) && (actorRef1 == actorRef2))
    {
        return false;
    }
    // Ensure that actors are either invalid (ie 'world') or valid to simulate.
    bool bActor1Valid = false;
    bool bActor2Valid = false;
    FPhysicsCommand::ExecuteRead(
        actorRef1,
        actorRef2,
        [&bActor1Valid, &bActor2Valid](const FPhysicsActorHandle& InActor1, const FPhysicsActorHandle& InActor2)
        {
            bActor1Valid = !FPhysicsInterface::IsValid(InActor1) || FPhysicsInterface::CanSimulate_AssumesLocked(InActor1);
            bActor2Valid = !FPhysicsInterface::IsValid(InActor2) || FPhysicsInterface::CanSimulate_AssumesLocked(InActor2);
        });
    if (false == (bActor1Valid && bActor2Valid))
    {
        OutActorRef1 = FPhysicsActorHandle();
        OutActorRef2 = FPhysicsActorHandle();
        return false;
    }
    OutActorRef1 = actorRef1;
    OutActorRef2 = actorRef2;
    return true;
}
FString URRUObjectUtils::GetSegMaskDepthStencilsAsText(ARRMeshActor* InActor)
{
    TArray<uint8> depthStencilValueList;

    // The validity and uniqueness of [meshComp's CustomDepthStencilValue] should have been already verified
    for (const auto& meshComp : InActor->MeshCompList)
    {
        depthStencilValueList.AddUnique(static_cast<uint8>(meshComp->CustomDepthStencilValue));
    }

    return FString::JoinBy(
        depthStencilValueList, TEXT("/"), [](const uint8& InDepthStencilValue) { return FString::FromInt(InDepthStencilValue); });
}
UMaterialInstanceDynamic* URRUObjectUtils::CreateMeshCompMaterialInstance(UMeshComponent* InMeshComp,
                                                                          int32 InMaterialIndex,
                                                                          const FString& InMaterialInterfaceName)
{
    verify(IsValid(InMeshComp));
    const FString& dynamicMaterialName = FString::Printf(TEXT("%s%s"), *InMeshComp->GetName(), *InMaterialInterfaceName);
    return InMeshComp->CreateDynamicMaterialInstance(
        InMaterialIndex, URRGameSingleton::Get()->GetMaterial(InMaterialInterfaceName), FName(*dynamicMaterialName));
}

bool URRUObjectUtils::ApplyMeshActorMaterialProps(ARRMeshActor* InMeshActor, const FRRMaterialProperty& InMaterialInfo)
{
    UMaterialInstanceDynamic* baseMaterial = Cast<UMaterialInstanceDynamic>(InMeshActor->BaseMeshComp->GetMaterial(0));
    if (baseMaterial)
    {
        URRGameSingleton* gameSingleton = URRGameSingleton::Get();
        if (false == InMaterialInfo.AlbedoTextureName.IsEmpty())
        {
            baseMaterial->SetTextureParameterValue(TEXT("AlbedoTexture"),
                                                   gameSingleton->GetTexture(InMaterialInfo.AlbedoTextureName));
        }
        if (false == InMaterialInfo.ORMTextureName.IsEmpty())
        {
            baseMaterial->SetTextureParameterValue(TEXT("MergeMapInput"), gameSingleton->GetTexture(InMaterialInfo.ORMTextureName));
        }
        if (false == InMaterialInfo.NormalTextureName.IsEmpty())
        {
            baseMaterial->SetTextureParameterValue(TEXT("MainNormalInput"),
                                                   gameSingleton->GetTexture(InMaterialInfo.NormalTextureName));
        }
        if (false == InMaterialInfo.MaskTextureName.IsEmpty())
        {
            baseMaterial->SetTextureParameterValue(TEXT("MaskSelection"),
                                                   gameSingleton->GetTexture(InMaterialInfo.MaskTextureName));
        }
        if (FLinearColor::Transparent != InMaterialInfo.ColorAlbedo)
        {
            baseMaterial->SetVectorParameterValue(TEXT("ColorAlbedo"), InMaterialInfo.ColorAlbedo);
        }
        return true;
    }
    return false;
}
