// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Drives/RobotVehicleMovementComponent.h"

// UE
#include "Algo/MinElement.h"
#include "CollisionQueryParams.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetMathLibrary.h"
#include "Net/UnrealNetwork.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Drives/RRFloatingMovementComponent.h"
#include "Robots/RRBaseRobot.h"

// rclUE
#include "rclcUtilities.h"

void URobotVehicleMovementComponent::Initialize()
{
    OwnerVehicle = CastChecked<ARRBaseRobot>(GetOwner());
    InitData();
}

void URobotVehicleMovementComponent::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URobotVehicleMovementComponent, OwnerVehicle);
}

void URobotVehicleMovementComponent::InitAIMovementComp()
{
    // NOTE: By default [AIMovementComp] is NOT auto-instantiated, leaving its creation to owner robot's decision
    if (nullptr == AIMovementComp)
    {
        AIMovementComp = URRUObjectUtils::CreateChildComponent<URRFloatingMovementComponent>(
            GetOwner(), *FString::Printf(TEXT("%sAIMoveComp"), *GetName()));
    }
}

void URobotVehicleMovementComponent::SetUpdatedComponent(USceneComponent* InNewUpdatedComponent)
{
    Super::SetUpdatedComponent(InNewUpdatedComponent);
    if (AIMovementComp)
    {
        AIMovementComp->SetUpdatedComponent(InNewUpdatedComponent);
    }
}

void URobotVehicleMovementComponent::TickComponent(float InDeltaTime,
                                                   enum ELevelTick TickType,
                                                   FActorComponentTickFunction* ThisTickFunction)
{
    if (!ShouldSkipUpdate(InDeltaTime))
    {
        if (AIMovementComp)
        {
            // Let the tick handled by [AIMovementComp] update [UpdatedComponent]'s velocity
            // NOTE:[AIMovementComp->UpdatedComponent & GetPawnOwner()] could be assigned after the 1st tick
            return;
        }
        else
        {
            Super::TickComponent(InDeltaTime, TickType, ThisTickFunction);
            // Make sure that everything is still valid, and that we are allowed to move.
            if (IsValid(UpdatedComponent))
            {
                //1- Update vels to OwnerVehicle's target vels
                if (OwnerVehicle)
                {
                    Velocity = OwnerVehicle->TargetLinearVel;
                    AngularVelocity = OwnerVehicle->TargetAngularVel;
                }
                else
                {
                    UE_LOG_WITH_INFO_NAMED_THROTTLE(5, LogLastHit, LogTemp, Warning, TEXT("OwnerVehicle is nullptr."));
                }

                //2- Movement control for OwnerVehicle
                UpdateMovement(InDeltaTime);

                //3- Update OwnerVehicle's velocity to [Velocity], must be after [UpdateMovement()]
                UpdateComponentVelocity();
            }
        }
    }
}

void URobotVehicleMovementComponent::UpdateMovement(float InDeltaTime)
{
    if (AIMovementComp)
    {
        // Let [AIMovementComp] drive the movement
        return;
    }

    const FQuat oldRotation = UpdatedComponent->GetComponentQuat();

    const FVector position = UpdatedComponent->ComponentVelocity * InDeltaTime;
    const float angularVelRad = FMath::DegreesToRadians(AngularVelocity.Z);
    const FQuat deltaRotation(FVector::ZAxisVector, InversionFactor * angularVelRad * InDeltaTime);

    DesiredRotation = oldRotation * deltaRotation;
    DesiredMovement = (oldRotation * position);

    // if Robot is on a moving platform, add the platform motion
    if (MovingPlatform != nullptr && bAdaptToSurfaceBelow)
    {
        FVector currentPlatformLocation = MovingPlatform->GetActorLocation();
        FVector platformTranslation = currentPlatformLocation - LastPlatformLocation;
        DesiredMovement += platformTranslation;

        // TODO : add platform rotation in DesiredRotation

        LastPlatformLocation = currentPlatformLocation;
    }

    // if testing collisions :
    // should test once with the desired motion, to check for blocking collisions, and once again after motion by contact points
    // modification (if some modifications!) if collision detected, need to check actions possible : stop motion (if static object
    // or object mass >> vehicle mass), reduce motion (if movable object and object mass ~ vehicle mass), ignore ?

    FHitResult hit;
    SafeMoveUpdatedComponent(DesiredMovement, DesiredRotation, true, hit);

    // If we bumped into something, try to slide along it
    if (hit.IsValidBlockingHit())
    {
        SlideAlongSurface(DesiredMovement, 1.0f - hit.Time, hit.Normal, hit);
    }

    if (bAdaptToSurfaceBelow)
    {
        // check for floor configuration beneath the robot : slopes, etc
        AActor* owner = GetOwner();
        FCollisionQueryParams traceParams = FCollisionQueryParams(FName(TEXT("Contact_Trace")), true, owner);
        traceParams.bReturnPhysicalMaterial = true;
        traceParams.bTraceComplex = true;
        traceParams.bReturnFaceIndex = true;
        traceParams.AddIgnoredActor(owner);

        // If few contact points defined, cast a single ray beneath the robot to get the floor orientation
        if (ContactPoints.Num() < 3)
        {
            // robot will be oriented as the normal vector in floor plane
            FVector startPos = owner->GetActorLocation() + FVector(0., 0., RayOffsetUp);
            FVector endPos = owner->GetActorLocation() - FVector(0., 0., RayOffsetDown);
            bool bIsFloorHit = GetWorld()->LineTraceSingleByChannel(hit,
                                                                    startPos,
                                                                    endPos,
                                                                    ECollisionChannel::ECC_Visibility,
                                                                    traceParams,
                                                                    FCollisionResponseParams::DefaultResponseParam);
            if (bIsFloorHit)
            {
                FVector forwardProjection = FVector::VectorPlaneProject(owner->GetActorForwardVector(), hit.ImpactNormal);
                owner->SetActorRotation(UKismetMathLibrary::MakeRotFromXZ(forwardProjection, hit.ImpactNormal));
                if (MovingPlatform == nullptr)
                {
                    FVector heightVariation = {0., 0., MinDistanceToFloor - hit.Distance};
                    owner->AddActorWorldOffset(heightVariation, true, &hit, ETeleportType::None);
                }
            }
            else
            {
                // very basic robot falling
                owner->AddActorWorldOffset(FVector(0., 0., -FallingSpeed * InDeltaTime), true, &hit, ETeleportType::None);
            }
        }
        else
        {
            // compute all impact points and keep the 3 closest
            // get the normal vector of the plane formed by these 3 points
            FVector contacts[3];
            float contactsDistance[3];
            uint8 nbContact = 0;

            for (USceneComponent* contact : ContactPoints)
            {
                // get the contact points
                FVector startPos = contact->GetComponentLocation() + FVector(0.f, 0.f, RayOffsetUp);
                FVector endPos = contact->GetComponentLocation() - FVector(0.f, 0.f, RayOffsetDown);
                bool bIsFloorHit = GetWorld()->LineTraceSingleByChannel(hit,
                                                                        startPos,
                                                                        endPos,
                                                                        ECollisionChannel::ECC_Visibility,
                                                                        traceParams,
                                                                        FCollisionResponseParams::DefaultResponseParam);
                if (!bIsFloorHit)
                {
                    // if no impact below, just consider this contact point is falling
                    hit.ImpactPoint = contact->GetComponentLocation() - FVector(0.f, 0.f, FallingSpeed * InDeltaTime);
                    hit.Distance = RayOffsetUp + FallingSpeed * InDeltaTime;
                }

                // keep only the 3 contacts with shortest distances
                if (nbContact < 3)
                {
                    contacts[nbContact] = hit.ImpactPoint;
                    contactsDistance[nbContact] = hit.Distance;
                    nbContact++;
                }
                else
                {
                    uint8 maxContactDistanceIndex = 0;
                    if (contactsDistance[1] > contactsDistance[0])
                        maxContactDistanceIndex = 1;
                    if (contactsDistance[2] > contactsDistance[maxContactDistanceIndex])
                        maxContactDistanceIndex = 2;
                    if (hit.Distance < contactsDistance[maxContactDistanceIndex])
                    {
                        contactsDistance[maxContactDistanceIndex] = hit.Distance;
                        contacts[maxContactDistanceIndex] = hit.ImpactPoint;
                    }
                }
            }

            // get the normal vector of the plane going through these 3 points
            FVector planeNormal = FVector::CrossProduct(contacts[1] - contacts[0], contacts[2] - contacts[0]);
            planeNormal.Normalize(0.01f);
            if (planeNormal.Z < 0.f)
                planeNormal = -planeNormal;

            FVector ForwardProjection = FVector::VectorPlaneProject(owner->GetActorForwardVector(), planeNormal);
            owner->SetActorRotation(UKismetMathLibrary::MakeRotFromXZ(ForwardProjection, planeNormal));

            if (MovingPlatform == nullptr)
            {
                // Moves the robot up or down, depending on impact position
                float minDistance = *Algo::MinElement(contactsDistance);
                minDistance -= RayOffsetUp;
                minDistance = FMath::Min(minDistance, FallingSpeed * InDeltaTime);

                FVector heightVariation = {0., 0., -minDistance};
                owner->AddActorWorldOffset(heightVariation, false, &hit, ETeleportType::None);
            }
        }
    }
}

void URobotVehicleMovementComponent::InitData()
{
    AActor* owner = GetOwner();
    ContactPoints.Empty();
    TArray<UActorComponent*> actorContactPoints = owner->GetComponentsByTag(USceneComponent::StaticClass(), TEXT("ContactPoint"));
    for (const auto& acp : actorContactPoints)
    {
        ContactPoints.Add(Cast<USceneComponent>(acp));
    }
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Nb Contact Points : %d"), ContactPoints.Num());
#endif

    // Compute the starting distance between the robot root and the floor
    // We consider that the robot is on a horizontal floor at the beginning
    FCollisionQueryParams traceParams = FCollisionQueryParams(FName(TEXT("Contact_Trace")), true, owner);
    traceParams.bReturnPhysicalMaterial = true;
    traceParams.bTraceComplex = true;
    traceParams.bReturnFaceIndex = true;
    traceParams.AddIgnoredActor(owner);

    FVector startPos = owner->GetActorLocation() + FVector(0.f, 0.f, 10.f);
    FVector endPos = owner->GetActorLocation() - FVector(0.f, 0.f, 50.f);
    FHitResult hitResult;
    bool bIsFloorHit = GetWorld()->LineTraceSingleByChannel(hitResult,
                                                            startPos,
                                                            endPos,
                                                            ECollisionChannel::ECC_Visibility,
                                                            traceParams,
                                                            FCollisionResponseParams::DefaultResponseParam);
    if (bIsFloorHit)
    {
        MinDistanceToFloor = hitResult.Distance;
    }
#if RAPYUTA_SIM_VERBOSE
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Min Distance To Floor = %f"), MinDistanceToFloor);
#endif
}

void URobotVehicleMovementComponent::SetMovingPlatform(AActor* InPlatform)
{
    MovingPlatform = InPlatform;
    LastPlatformLocation = InPlatform->GetActorLocation();
    LastPlatformRotation = InPlatform->GetActorQuat();
}

bool URobotVehicleMovementComponent::IsOnMovingPlatform()
{
    return MovingPlatform != nullptr;
}

void URobotVehicleMovementComponent::RemoveMovingPlatform()
{
    MovingPlatform = nullptr;
}
