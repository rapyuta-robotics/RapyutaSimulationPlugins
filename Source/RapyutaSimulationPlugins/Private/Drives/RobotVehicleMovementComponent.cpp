// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RobotVehicleMovementComponent.h"

// UE
#include "Algo/MinElement.h"
#include "CollisionQueryParams.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetMathLibrary.h"

void URobotVehicleMovementComponent::BeginPlay()
{
    Super::BeginPlay();
    InitMovementComponent();
}

void URobotVehicleMovementComponent::Initialize()
{
    GaussianRNGPosition = std::normal_distribution<>{NoiseMeanPos, NoiseVariancePos};
    GaussianRNGRotation = std::normal_distribution<>{NoiseMeanRot, NoiseVarianceRot};

    InitOdom();
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
        FCollisionQueryParams traceParams = FCollisionQueryParams(FName(TEXT("Contact_Trace")), true, PawnOwner);
        traceParams.bReturnPhysicalMaterial = true;
        traceParams.bTraceComplex = true;
        traceParams.bReturnFaceIndex = true;
        traceParams.AddIgnoredActor(PawnOwner);

        // If few contact points defined, cast a single ray beneath the robot to get the floor orientation
        if (ContactPoints.Num() < 3)
        {
            // robot will be oriented as the normal vector in floor plane
            FVector startPos = PawnOwner->GetActorLocation() + FVector(0., 0., RayOffsetUp);
            FVector endPos = PawnOwner->GetActorLocation() - FVector(0., 0., RayOffsetDown);
            bool bIsFloorHit = GetWorld()->LineTraceSingleByChannel(hit,
                                                                    startPos,
                                                                    endPos,
                                                                    ECollisionChannel::ECC_Visibility,
                                                                    traceParams,
                                                                    FCollisionResponseParams::DefaultResponseParam);
            if (bIsFloorHit)
            {
                FVector forwardProjection = FVector::VectorPlaneProject(PawnOwner->GetActorForwardVector(), hit.ImpactNormal);
                PawnOwner->SetActorRotation(UKismetMathLibrary::MakeRotFromXZ(forwardProjection, hit.ImpactNormal));
                if (MovingPlatform == nullptr)
                {
                    FVector heightVariation = {0., 0., MinDistanceToFloor - hit.Distance};
                    PawnOwner->AddActorWorldOffset(heightVariation, true, &hit, ETeleportType::None);
                }
            }
            else
            {
                // very basic robot falling
                PawnOwner->AddActorWorldOffset(
                    FVector(0., 0., -FallingSpeed * InDeltaTime), true, &hit, ETeleportType::None);
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

            FVector ForwardProjection = FVector::VectorPlaneProject(PawnOwner->GetActorForwardVector(), planeNormal);
            PawnOwner->SetActorRotation(UKismetMathLibrary::MakeRotFromXZ(ForwardProjection, planeNormal));

            if (MovingPlatform == nullptr)
            {
                // Moves the robot up or down, depending on impact position
                float minDistance = *Algo::MinElement(contactsDistance);
                minDistance -= RayOffsetUp;

                minDistance = FMath::Min(minDistance, FallingSpeed * InDeltaTime);

                FVector heightVariation = {0., 0., -minDistance};
                PawnOwner->AddActorWorldOffset(heightVariation, true, &hit, ETeleportType::None);
            }
        }
    }
}

void URobotVehicleMovementComponent::SetFrameIds(const FString& InFrameId, const FString& InChildFrameId)
{
    OdomData.header_frame_id = FrameId = InFrameId;
    OdomData.child_frame_id = ChildFrameId = InChildFrameId;
}

// todo separate ROS
void URobotVehicleMovementComponent::InitOdom()
{
    AActor* owner = GetOwner();
    OdomData.header_frame_id = FrameId;
    OdomData.child_frame_id = ChildFrameId;

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

    OdomData.pose_pose_position_x = InitialTransform.GetTranslation().X;
    OdomData.pose_pose_position_y = InitialTransform.GetTranslation().Y;
    OdomData.pose_pose_position_z = InitialTransform.GetTranslation().Z;
    OdomData.pose_pose_orientation = InitialTransform.GetRotation();

    PreviousTransform = InitialTransform;

    // todo temporary hardcoded
    OdomData.pose_covariance.Init(0, 36);
    OdomData.pose_covariance[0] = 1e-05f;
    OdomData.pose_covariance[7] = 1e-05f;
    OdomData.pose_covariance[14] = 1e+12;
    OdomData.pose_covariance[21] = 1e+12;
    OdomData.pose_covariance[28] = 1e+12;
    OdomData.pose_covariance[35] = 1e-03f;

    OdomData.twist_covariance.Init(0, 36);
    OdomData.twist_covariance[0] = 1e-05f;
    OdomData.twist_covariance[7] = 1e-05f;
    OdomData.twist_covariance[14] = 1e+12;
    OdomData.twist_covariance[21] = 1e+12;
    OdomData.twist_covariance[28] = 1e+12;
    OdomData.twist_covariance[35] = 1e-03f;

    IsOdomInitialized = true;
}

void URobotVehicleMovementComponent::UpdateOdom(float InDeltaTime)
{
    if (!IsOdomInitialized)
    {
        InitOdom();
    }

    // time
    float timeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    OdomData.header_stamp_sec = static_cast<int32>(timeNow);
    uint64 ns = (uint64)(timeNow * 1e+09f);
    OdomData.header_stamp_nanosec = static_cast<uint32>(ns - (OdomData.header_stamp_sec * 1e+09));

    // previous estimated data
    FVector previousEstimatedPos =
        FVector(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z);
    FQuat previousEstimatedRot = OdomData.pose_pose_orientation;

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

    OdomData.pose_pose_position_x = pos.X;
    OdomData.pose_pose_position_y = pos.Y;
    OdomData.pose_pose_position_z = pos.Z;
    OdomData.pose_pose_orientation = rot;

    OdomData.twist_twist_linear = OdomData.pose_pose_orientation.UnrotateVector(pos - previousEstimatedPos) / InDeltaTime;
    OdomData.twist_twist_angular =
        FMath::DegreesToRadians((rot * previousEstimatedRot.Inverse()).GetNormalized().Euler()) / InDeltaTime;
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
            UpdateMovement(InDeltaTime);
            UpdateOdom(InDeltaTime);
        }

        UpdateComponentVelocity();
    }
}

FTransform URobotVehicleMovementComponent::GetOdomTF() const
{
    return FTransform(OdomData.pose_pose_orientation,
                      FVector(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z));
}

void URobotVehicleMovementComponent::InitMovementComponent()
{
    InitOdom();

    ContactPoints.Empty();
    TArray<UActorComponent*> actorContactPoints = PawnOwner->GetComponentsByTag(USceneComponent::StaticClass(), "ContactPoint");
    for (auto acp : actorContactPoints)
    {
        USceneComponent* scp = Cast<USceneComponent>(acp);
        ContactPoints.Add(scp);
    }
    UE_LOG(LogTemp,
           Warning,
           TEXT("URobotVehicleMovementComponent::InitMovementComponent - Nb Contact Points : %d"),
           ContactPoints.Num());

    // Compute the starting distance between the robot root and the floor
    // We consider that the robot is on a horizontal floor at the beginning
    FCollisionQueryParams traceParams = FCollisionQueryParams(FName(TEXT("Contact_Trace")), true, PawnOwner);
    traceParams.bReturnPhysicalMaterial = true;
    traceParams.bTraceComplex = true;
    traceParams.bReturnFaceIndex = true;
    traceParams.AddIgnoredActor(PawnOwner);

    FVector startPos = PawnOwner->GetActorLocation() + FVector(0.f, 0.f, 10.f);
    FVector endPos = PawnOwner->GetActorLocation() - FVector(0.f, 0.f, 50.f);
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
        UE_LOG(LogTemp,
               Warning,
               TEXT("URobotVehicleMovementComponent::InitMovementComponent - Min Distance To Floor = %f"),
               MinDistanceToFloor);
    }
}

void URobotVehicleMovementComponent::SetMovingPlatform(AActor* InPlatform)
{
    // UE_LOG(LogTemp, Warning, TEXT("URobotVehicleMovementComponent::SetMovingPlatform..."));
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
    // UE_LOG(LogTemp, Warning, TEXT("URobotVehicleMovementComponent::RemoveMovingPlatform..."));
    MovingPlatform = nullptr;
}
