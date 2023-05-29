#include "Core/RRUObjectUtils.h"

// UE
#include "Components/ActorComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "GameFramework/Actor.h"

// RapyutaSimulationPlugins
#include "Core/RRBaseActor.h"
#include "Core/RRGameSingleton.h"
#include "Core/RRMathUtils.h"
#include "Core/RRMeshActor.h"

//! Hardcoded [DATA_SYNTH_CUSTOM_DEPTH_STENCILS], due to UE5 mismatched pixels in captured image vs ones written into the custom depth buffer
//! https://superyateam.com/2019/06/17/custom-depth-and-custom-depth-stencil-in-ue4/
static TArray<uint8> DATA_SYNTH_CUSTOM_DEPTH_STENCILS = []()
{
    TArray<uint8> data = {
        /*0*/ 0,
        /*1*/ 13,
        /*2*/ 22,
        /*3*/ 28,
        /*4*/ 34,
        /*5*/ 38,
        /*6*/ 42,
        /*7*/ 46,
        /*8*/ 49,
        /*9*/ 53,

        /*10*/ 56,
        /*11*/ 58,
        /*12*/ 61,
        /*13*/ 64,
        /*14*/ 66,
        /*15*/ 68,
        /*16*/ 71,
        /*17*/ 73,
        /*18*/ 75,
        /*19*/ 77,

        /*20*/ 79,
        /*21*/ 81,
        /*22*/ 83,
        /*23*/ 85,
        /*24*/ 86,
        /*25*/ 88,
        /*26*/ 90,
        /*27*/ 91,
        /*28*/ 93,
        /*29*/ 95,

        /*30*/ 96,
        /*31*/ 98,
        /*32*/ 99,
        /*33*/ 101,
        /*34*/ 102,
        /*35*/ 103,
        /*36*/ 105,
        /*37*/ 106,
        /*38*/ 107,
        /*39*/ 109,

        /*40*/ 110,
        /*41*/ 111,
        /*42*/ 113,
        /*43*/ 114,
        /*44*/ 115,
        /*45*/ 116,
        /*46*/ 118,
        /*47*/ 119,
        /*48*/ 120,
        /*49*/ 121,

        /*50*/ 122,
        /*51*/ 123,
        /*52*/ 124,
        /*53*/ 126,
        /*54*/ 127,
        /*55*/ 128,
        /*56*/ 129,
        /*57*/ 130,
        /*58*/ 131,
        /*59*/ 132,

        /*60*/ 133,
        /*61*/ 134,
        /*62*/ 135,
        /*63*/ 136,
        /*64*/ 137,
        /*65*/ 138,
        /*66*/ 139,
        /*67*/ 140,
        /*68*/ 141,
        /*69*/ 142,

        /*70*/ 143,
        /*71*/ 144,
        /*72*/ 145,
        /*73*/ 146,
        /*74*/ 147,
        /*75*/ 148,
    };

    for (auto i = 76; i <= 255; ++i)
    {
        data.Add(0);
    }
    return data;
}();

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

USceneComponent* URRUObjectUtils::SetupDefaultRootComponent(AActor* InActor)
{
    if (InActor->GetRootComponent())
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("[%s] RootComponent has already been set up!"), *InActor->GetName());
        return InActor->GetRootComponent();
    }

    USceneComponent* defaultRoot = CreateSelfSubobject<USceneComponent>(InActor, TEXT("DefaultRoot"));

    if (!(defaultRoot && InActor->SetRootComponent(defaultRoot)))
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("[%s] Failed setting up root component!"), *InActor->GetName());
        return nullptr;
    }
#if WITH_EDITOR
    defaultRoot->bVisualizeComponent = false;
#endif
    return defaultRoot;
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
            UE_LOG_WITH_INFO(LogRapyutaCore,
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
    if (InWorld->IsNetMode(NM_Standalone))
    {
        spawnInfo.Name = InActorName.IsEmpty() ? FName(*FString::Printf(TEXT("%d_%s"), InSceneInstanceId, *InActorClass->GetName()))
                                               : FName(*InActorName);
    }
    // else let client auto assign a unique name
    spawnInfo.SpawnCollisionHandlingOverride = InCollisionHandlingType;
    ARRBaseActor* newActor = InWorld->SpawnActor<ARRBaseActor>(InActorClass, InActorTransform, spawnInfo);
    if (newActor)
    {
        // This is set in ctor
        ensure(newActor->SceneInstanceId == InSceneInstanceId);
        newActor->SetEntityModelName(InEntityModelName);

#if WITH_EDITOR
        // [newObjectActor] ActorUniqueName is set inside the CTOR
        // In Editor, Use the id itself for actor's label as well, just for sake of verification.
        newActor->SetActorLabel(spawnInfo.Name.ToString());
#endif

        // Initializing itself
        ensure(newActor->Initialize());
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore,
                         Error,
                         TEXT("SceneInstance[%d] Failed spawning actor [%s] of model [%d] as class[%s]"),
                         InSceneInstanceId,
                         *InActorName,
                         *InEntityModelName,
                         *InActorClass->GetName());
    }

    return newActor;
}

void URRUObjectUtils::GetActorCenterAndBoundingBoxVertices(const AActor* InActor,
                                                           const AActor* InBaseActor,
                                                           TArray<FVector>* OutCenterAndVertices3D,
                                                           bool bInIncludeNonColliding)
{
    OutCenterAndVertices3D->Reset();
    ARRGameState* gameState = URRCoreUtils::GetGameState<ARRGameState>(InActor);

    // [InActor]'s Local bounding box
    //NOTE: For a single [InActor], its EXTENT would be the same as using [InActor->GetComponentsBoundingBox()]
    FVector centerLocal, extentsLocal;
    FBox actorLocalBox = InActor->CalculateComponentsBoundingBoxInLocalSpace(bInIncludeNonColliding);
    actorLocalBox.GetCenterAndExtents(centerLocal, extentsLocal);

    // [InActor]'s global oriented bounding box with rotation (in BaseActor frame or World frame)
    // Reference : DrawDebugBox()
    // (snote) The exact order of points is important. Consult with Robot Research team before adjusting.

    // Transfom [centerLocal] to global
    // UnrealEngine/Engine/Plugins/2D/Paper2D/Source/Paper2D/Private/PaperSpriteComponent.cpp:179
    const FTransform baseTransform =
        InBaseActor ? InActor->GetActorTransform().GetRelativeTransform(InBaseActor->GetTransform()) : InActor->GetTransform();
    const FVector centerBase = baseTransform.TransformPosition(centerLocal);
    OutCenterAndVertices3D->Emplace(centerBase);

    // [InActor]'s global Rotation (in BaseActor frame or World frame)
    const FQuat& actorQuat = InBaseActor ? URRUObjectUtils::GetRelativeQuatFrom(InActor, InBaseActor) : InActor->GetActorQuat();

    // Calculate global vertices as : [center] + Rotated [ExtentsLocal]
    for (auto i = 0; i < gameState->ENTITY_BOUNDING_BOX_VERTEX_NORMALS.Num(); ++i)
    {
        OutCenterAndVertices3D->Emplace(
            centerBase + actorQuat.RotateVector(GetDirectedExtent(gameState->GetEntityBBVertexNormal(i), extentsLocal)));
    }
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
void URRUObjectUtils::SetHomoLinearConstraintMotion(FConstraintInstance* InCI, const ELinearConstraintMotion InHomoLinearMotion)
{
    const bool bEnabled = (InHomoLinearMotion != ELinearConstraintMotion::LCM_Locked);
    InCI->SetLinearPositionDrive(bEnabled, bEnabled, bEnabled);
    InCI->SetLinearVelocityDrive(bEnabled, bEnabled, bEnabled);
    InCI->SetLinearXMotion(InHomoLinearMotion);
    InCI->SetLinearYMotion(InHomoLinearMotion);
    InCI->SetLinearZMotion(InHomoLinearMotion);
}

void URRUObjectUtils::SetHomoAngularConstraintMotion(FConstraintInstance* InCI, const EAngularConstraintMotion InHomoAngularMotion)
{
    const bool bEnabled = (InHomoAngularMotion != EAngularConstraintMotion::ACM_Locked);
    InCI->SetOrientationDriveTwistAndSwing(bEnabled, bEnabled);
    InCI->SetAngularVelocityDriveTwistAndSwing(bEnabled, bEnabled);
    InCI->SetAngularSwing1Motion(InHomoAngularMotion);
    InCI->SetAngularSwing2Motion(InHomoAngularMotion);
    InCI->SetAngularTwistMotion(InHomoAngularMotion);
}

FString URRUObjectUtils::GetSegMaskDepthStencilsAsText(AActor* InActor)
{
    TArray<uint8> depthStencilValueList;

    if (ARRMeshActor* meshActor = Cast<ARRMeshActor>(InActor))
    {
        // The validity and uniqueness of [meshComp's CustomDepthStencilValue] should have been already verified
        for (const auto& meshComp : meshActor->MeshCompList)
        {
            depthStencilValueList.AddUnique(
                DATA_SYNTH_CUSTOM_DEPTH_STENCILS[static_cast<uint8>(meshComp->CustomDepthStencilValue)]);
        }
    }
    else if (AStaticMeshActor* staticMeshActor = Cast<AStaticMeshActor>(InActor))
    {
        depthStencilValueList.Add(DATA_SYNTH_CUSTOM_DEPTH_STENCILS[static_cast<uint8>(
            staticMeshActor->GetStaticMeshComponent()->CustomDepthStencilValue)]);
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

int32 URRUObjectUtils::GetActorMaterialsNum(AActor* InActor)
{
    if (auto* meshActor = Cast<ARRMeshActor>(InActor))
    {
        return meshActor->BaseMeshComp->GetMaterials().Num();
    }
    else if (auto* staticMeshActor = Cast<AStaticMeshActor>(InActor))
    {
        return staticMeshActor->GetStaticMeshComponent()->GetMaterials().Num();
    }
    return 0;
}

UMaterialInstanceDynamic* URRUObjectUtils::GetActorBaseMaterial(AActor* InActor, int32 InMaterialIndex)
{
    UMaterialInstanceDynamic* baseMaterial = nullptr;
    // NOTE: First mesh comp has more priority than primitive root comp
    if (auto* meshComp = Cast<UMeshComponent>(InActor->GetComponentByClass(UMeshComponent::StaticClass())))
    {
        baseMaterial = Cast<UMaterialInstanceDynamic>(meshComp->GetMaterial(InMaterialIndex));
        ensure(baseMaterial);
    }
    else if (auto* rootPrimitiveComp = Cast<UPrimitiveComponent>(InActor->GetRootComponent()))
    {
        baseMaterial = Cast<UMaterialInstanceDynamic>(rootPrimitiveComp->GetMaterial(InMaterialIndex));
    }
    return baseMaterial;
}

bool URRUObjectUtils::ApplyMeshActorMaterialProps(AActor* InActor,
                                                  const FRRMaterialProperty& InMaterialInfo,
                                                  bool bApplyManufacturingAlbedo)
{
    UMeshComponent* meshComp = nullptr;
    if (auto* meshActor = Cast<ARRMeshActor>(InActor))
    {
        meshComp = meshActor->BaseMeshComp;
    }
    else if (auto* staticMeshActor = Cast<AStaticMeshActor>(InActor))
    {
        meshComp = staticMeshActor->GetStaticMeshComponent();
    }
    else
    {
        meshComp = Cast<UMeshComponent>(InActor->GetComponentByClass(UMeshComponent::StaticClass()));
    }

    if (nullptr == meshComp)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("[%s] has NO Mesh component"), *InActor->GetName());
        return false;
    }
    for (auto i = 0; i < GetActorMaterialsNum(InActor); ++i)
    {
        UMaterialInstanceDynamic* material = Cast<UMaterialInstanceDynamic>(meshComp->GetMaterial(i));
        if (material)
        {
            ApplyMaterialProps(material, InMaterialInfo, bApplyManufacturingAlbedo);
            return true;
        }
    }
    return false;
}

void URRUObjectUtils::ApplyMaterialProps(UMaterialInstanceDynamic* InMaterial,
                                         const FRRMaterialProperty& InMaterialInfo,
                                         bool bApplyManufacturingAlbedo)
{
    URRGameSingleton* gameSingleton = URRGameSingleton::Get();
    UTexture* blackMaskTexture = gameSingleton->GetTexture(URRGameSingleton::TEXTURE_NAME_BLACK_MASK);
    UTexture* whiteMaskTexture = gameSingleton->GetTexture(URRGameSingleton::TEXTURE_NAME_WHITE_MASK);

    // Albedo texture
    if (bApplyManufacturingAlbedo)
    {
        if (InMaterialInfo.AlbedoTextureNameList.Num() > 0)
        {
            InMaterial->SetTextureParameterValue(
                FRRMaterialProperty::PROP_NAME_ALBEDO,
                gameSingleton->GetTexture(URRMathUtils::GetRandomElement(InMaterialInfo.AlbedoTextureNameList)));
        }
        // Albedo color
        InMaterial->SetVectorParameterValue(FRRMaterialProperty::PROP_NAME_COLOR_ALBEDO,
                                            (InMaterialInfo.AlbedoColorList.Num() > 0)
                                                ? URRMathUtils::GetRandomElement(InMaterialInfo.AlbedoColorList)
                                                : FLinearColor::Transparent);

        // Mask Texture: default White
        InMaterial->SetTextureParameterValue(FRRMaterialProperty::PROP_NAME_MASK,
                                             InMaterialInfo.MaskTextureName.IsEmpty()
                                                 ? whiteMaskTexture
                                                 : gameSingleton->GetTexture(InMaterialInfo.MaskTextureName));
    }
    else
    {
        // Mask Texture: default Black
        InMaterial->SetTextureParameterValue(FRRMaterialProperty::PROP_NAME_MASK, blackMaskTexture);
    }

    // ORM Texture
    if (false == InMaterialInfo.ORMTextureName.IsEmpty())
    {
        InMaterial->SetTextureParameterValue(FRRMaterialProperty::PROP_NAME_ORM,
                                             gameSingleton->GetTexture(InMaterialInfo.ORMTextureName));
    }

    // Normal Texture
    if (false == InMaterialInfo.NormalTextureName.IsEmpty())
    {
        InMaterial->SetTextureParameterValue(FRRMaterialProperty::PROP_NAME_NORMAL,
                                             gameSingleton->GetTexture(InMaterialInfo.NormalTextureName));
    }
}

bool URRUObjectUtils::SetMeshActorColor(AActor* InMeshActor, const FLinearColor& InColor, bool InEmitColor)
{
    UMeshComponent* meshComp = nullptr;
    if (auto* meshActor = Cast<ARRMeshActor>(InMeshActor))
    {
        meshComp = meshActor->BaseMeshComp;
    }
    else if (auto* staticMeshActor = Cast<AStaticMeshActor>(InMeshActor))
    {
        meshComp = staticMeshActor->GetStaticMeshComponent();
    }
    else
    {
        meshComp = Cast<UMeshComponent>(InMeshActor->GetComponentByClass(UMeshComponent::StaticClass()));
    }

    if (nullptr == meshComp)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("[%s] has NO Mesh component"), *InMeshActor->GetName());
        return false;
    }

    const TCHAR* maskTextureName = (FLinearColor::Transparent == InColor) ? URRGameSingleton::TEXTURE_NAME_WHITE_MASK
                                                                          : URRGameSingleton::TEXTURE_NAME_BLACK_MASK;
    URRGameSingleton* gameSingleton = URRGameSingleton::Get();
    UTexture* maskTexture = gameSingleton->GetTexture(maskTextureName);
    float emissiveStrength = InEmitColor ? 500.f : 0.f;

    for (auto i = 0; i < meshComp->GetMaterials().Num(); ++i)
    {
        UMaterialInstanceDynamic* material = Cast<UMaterialInstanceDynamic>(meshComp->GetMaterial(i));
        if (material)
        {
            material->SetTextureParameterValue(FRRMaterialProperty::PROP_NAME_MASK, maskTexture);
            material->SetVectorParameterValue(FRRMaterialProperty::PROP_NAME_COLOR_ALBEDO, InColor);
            material->SetScalarParameterValue(FRRMaterialProperty::PROP_NAME_EMISSIVE_STRENGTH, emissiveStrength);
        }
    }
    return true;
}

void URRUObjectUtils::RandomizeActorAppearance(AActor* InActor, const FRRTextureData& InTextureData)
{
    UMeshComponent* baseMeshComp = nullptr;
    if (auto* meshActor = Cast<ARRMeshActor>(InActor))
    {
        baseMeshComp = meshActor->BaseMeshComp;
    }
    else if (auto* staticMeshActor = Cast<AStaticMeshActor>(InActor))
    {
        baseMeshComp = staticMeshActor->GetStaticMeshComponent();
    }
    check(baseMeshComp);

    for (auto i = 0; i < baseMeshComp->GetMaterials().Num(); ++i)
    {
        UMaterialInstanceDynamic* baseMaterial = Cast<UMaterialInstanceDynamic>(baseMeshComp->GetMaterial(i));
        if (baseMaterial)
        {
#if 0    // Only required for MI_Floor material, which does not have texture randomization for now
            baseMaterial->ClearParameterValues();
#endif
        }
        else
        {
            baseMaterial = baseMeshComp->CreateAndSetMaterialInstanceDynamicFromMaterial(i, baseMeshComp->GetMaterial(i));
        }
        baseMaterial->SetTextureParameterValue(FRRMaterialProperty::PROP_NAME_ALBEDO, InTextureData.GetRandomTexture());
        baseMaterial->SetVectorParameterValue(FRRMaterialProperty::PROP_NAME_COLOR_ALBEDO, URRMathUtils::GetRandomColor());
    }
}
