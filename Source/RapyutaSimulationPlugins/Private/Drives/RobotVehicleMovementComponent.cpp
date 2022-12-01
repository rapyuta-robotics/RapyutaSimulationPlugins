// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RobotVehicleMovementComponent.h"

// UE
#include "Algo/MinElement.h"
#include "CollisionQueryParams.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetMathLibrary.h"
#include "Net/UnrealNetwork.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Robots/RRRobotBaseVehicle.h"

// rclUE
#include "rclcUtilities.h"

void URobotVehicleMovementComponent::Initialize()
{
    OwnerVehicle = CastChecked<ARRRobotBaseVehicle>(GetOwner());
    GaussianRNGPosition = std::normal_distribution<>{NoiseMeanPos, NoiseVariancePos};
    GaussianRNGRotation = std::normal_distribution<>{NoiseMeanRot, NoiseVarianceRot};

    InitData();
}

void URobotVehicleMovementComponent::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME(URobotVehicleMovementComponent, OwnerVehicle);
}

void URobotVehicleMovementComponent::UpdateMovement(float InDeltaTime)
{
    const FQuat oldRotation = UpdatedComponent->GetComponentQuat();

    FVector position = UpdatedComponent->ComponentVelocity * InDeltaTime;
    FQuat deltaRotation(FVector::ZAxisVector, InversionFactor * AngularVelocity.Z * InDeltaTime);

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

void URobotVehicleMovementComponent::SetFrameIds(const FString& InFrameId, const FString& InChildFrameId)
{
    OdomData.Header.FrameId = FrameId = InFrameId;
    OdomData.ChildFrameId = ChildFrameId = InChildFrameId;
}

// todo separate ROS
void URobotVehicleMovementComponent::InitOdom()
{
    AActor* owner = GetOwner();
    OdomData.Header.FrameId = FrameId;
    OdomData.ChildFrameId = ChildFrameId;

    if (OdomSource == EOdomSource::ENCODER)
    {    // odom source = encoder. Odom frame start from robot initial pose
        InitialTransform.SetTranslation(owner->GetActorLocation());
        InitialTransform.SetRotation(owner->GetActorQuat());
    }
    else
    {    // odom source = world. Odom frame start from world origin
        InitialTransform.SetTranslation(FVector::ZeroVector);
        InitialTransform.SetRotation(FQuat::Identity);
    }

    OdomData.Pose.Pose.Position = InitialTransform.GetTranslation();
    OdomData.Pose.Pose.Orientation = InitialTransform.GetRotation();

    PreviousTransform = InitialTransform;
    PreviousNoisyTransform = InitialTransform;

    // todo temporary hardcoded
    OdomData.Pose.Covariance[0] = 1e-05f;
    OdomData.Pose.Covariance[7] = 1e-05f;
    OdomData.Pose.Covariance[14] = 1e+12;
    OdomData.Pose.Covariance[21] = 1e+12;
    OdomData.Pose.Covariance[28] = 1e+12;
    OdomData.Pose.Covariance[35] = 1e-03f;

    OdomData.Twist.Covariance[0] = 1e-05f;
    OdomData.Twist.Covariance[7] = 1e-05f;
    OdomData.Twist.Covariance[14] = 1e+12;
    OdomData.Twist.Covariance[21] = 1e+12;
    OdomData.Twist.Covariance[28] = 1e+12;
    OdomData.Twist.Covariance[35] = 1e-03f;

    bIsOdomInitialized = true;
}

void URobotVehicleMovementComponent::UpdateOdom(float InDeltaTime)
{
    if (!bIsOdomInitialized)
    {
        InitOdom();
    }

    // time
    OdomData.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));

    // previous estimated data (with noise)
    FVector previousEstimatedPos = PreviousNoisyTransform.GetTranslation();
    FQuat previousEstimatedRot = PreviousNoisyTransform.GetRotation();

    AActor* owner = GetOwner();

    // position
    FVector pos = InitialTransform.GetRotation().UnrotateVector(owner->GetActorLocation() - InitialTransform.GetTranslation());
    FVector previousPos = PreviousTransform.GetTranslation();    // prev pos without noise
    PreviousTransform.SetTranslation(pos);
    pos += previousEstimatedPos - previousPos + WithNoise * FVector(GaussianRNGPosition(Gen), GaussianRNGPosition(Gen), 0);

    FRotator noiseRot = FRotator(0, 0, WithNoise * GaussianRNGRotation(Gen));
    FQuat rot = owner->GetActorQuat() * InitialTransform.GetRotation().Inverse();
    FQuat previousRot = PreviousTransform.GetRotation();
    PreviousTransform.SetRotation(rot);
    rot = noiseRot.Quaternion() * previousEstimatedRot * previousRot.Inverse() * rot;
    rot.Normalize();

    PreviousNoisyTransform.SetTranslation(pos);
    PreviousNoisyTransform.SetRotation(rot);

    OdomData.Pose.Pose.Position = pos + RootOffset.GetTranslation();
    OdomData.Pose.Pose.Orientation = rot;

    OdomData.Twist.Twist.Linear = OdomData.Pose.Pose.Orientation.UnrotateVector(pos - previousEstimatedPos) / InDeltaTime;
    OdomData.Twist.Twist.Angular =
        FMath::DegreesToRadians((rot * previousEstimatedRot.Inverse()).GetNormalized().Euler()) / InDeltaTime;

    OdomData.Pose.Pose.Orientation *= RootOffset.GetRotation();
}

void URobotVehicleMovementComponent::TickComponent(float InDeltaTime,
                                                   enum ELevelTick TickType,
                                                   FActorComponentTickFunction* ThisTickFunction)
{
    if (!ShouldSkipUpdate(InDeltaTime))
    {
        Super::TickComponent(InDeltaTime, TickType, ThisTickFunction);

        // Make sure that everything is still valid, and that we are allowed to move.
        if (IsValid(UpdatedComponent))
        {
            //1- Update vels to OwnerVehicle's target vels
            Velocity = OwnerVehicle->TargetLinearVel;
            AngularVelocity = OwnerVehicle->TargetAngularVel;

            //2- Movement control for OwnerVehicle
            UpdateMovement(InDeltaTime);
            UpdateOdom(InDeltaTime);

            //3- Update OwnerVehicle's velocity to [Velocity], must be after [UpdateMovement()]
            UpdateComponentVelocity();
        }
    }
}

FTransform URobotVehicleMovementComponent::GetOdomTF() const
{
    return FTransform(OdomData.Pose.Pose.Orientation, OdomData.Pose.Pose.Position);
}

void URobotVehicleMovementComponent::InitData()
{
    InitOdom();

    AActor* owner = GetOwner();
    ContactPoints.Empty();
    TArray<UActorComponent*> actorContactPoints = owner->GetComponentsByTag(USceneComponent::StaticClass(), TEXT("ContactPoint"));
    for (const auto& acp : actorContactPoints)
    {
        ContactPoints.Add(Cast<USceneComponent>(acp));
    }
#if RAPYUTA_SIM_DEBUG
    UE_LOG(LogRapyutaCore, Warning, TEXT("URobotVehicleMovementComponent::InitData - Nb Contact Points : %d"), ContactPoints.Num());
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
#if RAPYUTA_SIM_DEBUG
    UE_LOG(
        LogRapyutaCore, Warning, TEXT("URobotVehicleMovementComponent::InitData - Min Distance To Floor = %f"), MinDistanceToFloor);
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
