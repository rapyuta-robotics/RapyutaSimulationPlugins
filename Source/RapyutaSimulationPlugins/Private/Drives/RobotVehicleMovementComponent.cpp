// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Drives/RobotVehicleMovementComponent.h"

#include "CollisionQueryParams.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetMathLibrary.h"

void URobotVehicleMovementComponent::UpdateMovement(float DeltaTime)
{
    const FQuat OldRotation = UpdatedComponent->GetComponentQuat();

    FVector position = UpdatedComponent->ComponentVelocity * DeltaTime;
    FQuat DeltaRotation(FVector::ZAxisVector, AngularVelocity.Z * DeltaTime);

    DesiredRotation = OldRotation * DeltaRotation;
    DesiredMovement = (OldRotation * position);

    // if Robot is on a moving platform, add the platform motion
    if (MovingPlatform != nullptr && FollowFloor )
    {
        FVector CurrentPlatformLocation = FVector(MovingPlatform->GetActorLocation());
        FVector PlatformTranslation = CurrentPlatformLocation - LastPlatformLocation;
        DesiredMovement += PlatformTranslation;

        // TODO : add platform rotation in DesiredRotation
        
        LastPlatformLocation = FVector(CurrentPlatformLocation);
    }



    // if testing collisions :
    // should test once with the desired motion, to check for blocking collisions, and once again after motion by contact points modification (if some modifications!)
    // if collision detected, need to check actions possible : stop motion (if static object or object mass >> vehicle mass), reduce motion (if movable object and object mass ~ vehicle mass), ignore ?
    /*
    bool GetWorld()->ComponentOverlapMultiByChannel(
                                                    TArray< struct FOverlapResult >& O...,
                                                    const UPrimitiveComponent* Pri...,
                                                    const FVector& Pos,
                                                    const FQuat& Rot,
                                                    ECollisionChannel TraceChannel,
                                                    const FComponentQueryParams& Param...,
                                                    const FCollisionObjectQueryParams&...
                                                    );

    bool GetWorld()->ComponentSweepMulti(
                                        TArray< struct FHitResult >& OutHi...,
                                        UPrimitiveComponent* PrimComp,
                                        const FVector& Start,
                                        const FVector& End,
                                        const FQuat& Rot,
                                        const FComponentQueryParams& Param...
                                    );

    bool GetWorld()->SweepSingleByChannel(
                                        FHitResult& OutHit,
                                        const FVector& Start,
                                        const FVector& End,
                                        const FQuat& Rot,
                                        ECollisionChannel TraceChannel,
                                        const FCollisionShape& CollisionSh...,
                                        const FCollisionQueryParams& Param...,
                                        const FCollisionResponseParams& Re...
                                    );
    */

    FHitResult Hit;
    SafeMoveUpdatedComponent(DesiredMovement, DesiredRotation, true, Hit);

    // If we bumped into something, try to slide along it
    if (Hit.IsValidBlockingHit())
    {
        UE_LOG( LogTemp, Warning, TEXT("URobotVehicleMovementComponent::UpdateMovement -> BlockingHit - Hit.normal : %s"), *Hit.Normal.ToString() );
        SlideAlongSurface(DesiredMovement, 1.0f - Hit.Time, Hit.Normal, Hit);
    }

    if( FollowFloor )
    {
        // check for floor configuration beneath the robot : slopes, etc
        FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Contact_Trace")), true, PawnOwner);
        TraceParams.bReturnPhysicalMaterial = true;
        TraceParams.bTraceComplex = true;
        TraceParams.bReturnFaceIndex = true;
        TraceParams.AddIgnoredActor(PawnOwner);

        // If few contact points defined, cast a single ray beneath the robot to get the floor orientation
        if (ContactPoints.Num() < 3)
        {
            // robot will be oriented as the normal vector in floor plane
            FVector startPos = PawnOwner->GetActorLocation() + FVector(0., 0., RayOffsetUp);
            FVector endPos = PawnOwner->GetActorLocation() - FVector(0., 0., RayOffsetDown);
            bool IsFloorHit = GetWorld()->LineTraceSingleByChannel(
                Hit, startPos, endPos, ECollisionChannel::ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);
            if (IsFloorHit)
            {
                FVector ForwardProjection = FVector::VectorPlaneProject(PawnOwner->GetActorForwardVector(), Hit.ImpactNormal);
                PawnOwner->SetActorRotation(UKismetMathLibrary::MakeRotFromXZ(ForwardProjection, Hit.ImpactNormal));
                if (MovingPlatform == nullptr)
                {
                    FVector HeightVariation = {0., 0., MinDistanceToFloor - Hit.Distance};
                    PawnOwner->AddActorWorldOffset(HeightVariation, true, &Hit, ETeleportType::TeleportPhysics);
                    // UE_LOG(LogTemp, Warning, TEXT("URobotVehicleMovementComponent::UpdateMovement - Distance Modification = %f"),
                    // HeightVariation.Z );
                }
            }
            else
            {
                // very basic robot falling
                PawnOwner->AddActorWorldOffset(FVector(0., 0., -FallingSpeed * DeltaTime), true, &Hit, ETeleportType::TeleportPhysics);
            }
        }
        else
        {
            // compute all impact points and keep the 3 closest
            // get the normal vector of the plane formed by these 3 points
            FVector Contacts[3];
            float ContactsDistance[3];
            uint8 NbContact = 0;

            for (USceneComponent* Contact : ContactPoints)
            {
                // get the contact points
                // UE_LOG( LogTemp, Warning, TEXT("URobotVehicleMovementComponent::UpdateMovement - Contact Point = %f %f %f"),
                // Contact->GetComponentLocation().X, Contact->GetComponentLocation().Y, Contact->GetComponentLocation().Z );
                FVector startPos = Contact->GetComponentLocation() + FVector(0., 0., RayOffsetUp);
                FVector endPos = Contact->GetComponentLocation() - FVector(0., 0., RayOffsetDown);
                bool IsFloorHit = GetWorld()->LineTraceSingleByChannel(Hit,
                                                                    startPos,
                                                                    endPos,
                                                                    ECollisionChannel::ECC_Visibility,
                                                                    TraceParams,
                                                                    FCollisionResponseParams::DefaultResponseParam);
                if (!IsFloorHit)
                {
                    // if no impact below, just consider this contact point is falling
                    Hit.ImpactPoint = Contact->GetComponentLocation() - FVector(0., 0., FallingSpeed * DeltaTime);
                    Hit.Distance = RayOffsetUp + FallingSpeed * DeltaTime;
                }

                // keep only the 3 contacts with shortest distances
                if (NbContact < 3)
                {
                    Contacts[NbContact] = Hit.ImpactPoint;
                    ContactsDistance[NbContact] = Hit.Distance;
                    NbContact++;
                }
                else
                {
                    uint8 MaxContactDistanceIndex = 0;
                    if (ContactsDistance[1] > ContactsDistance[0])
                        MaxContactDistanceIndex = 1;
                    if (ContactsDistance[2] > ContactsDistance[MaxContactDistanceIndex])
                        MaxContactDistanceIndex = 2;
                    if (Hit.Distance < ContactsDistance[MaxContactDistanceIndex])
                    {
                        ContactsDistance[MaxContactDistanceIndex] = Hit.Distance;
                        Contacts[MaxContactDistanceIndex] = Hit.ImpactPoint;
                    }
                }
            }

            // get the normal vector of the plane going through these 3 points
            FVector PlaneNormal = FVector::CrossProduct(Contacts[1] - Contacts[0], Contacts[2] - Contacts[0]);
            PlaneNormal.Normalize(0.01);
            if (PlaneNormal.Z < 0.)
                PlaneNormal = -PlaneNormal;
            // UE_LOG( LogTemp, Warning, TEXT("URobotVehicleMovementComponent::UpdateMovement - PlaneNormal = %f %f %f"), PlaneNormal.X,
            // PlaneNormal.Y, PlaneNormal.Z );

            FVector ForwardProjection = FVector::VectorPlaneProject(PawnOwner->GetActorForwardVector(), PlaneNormal);
            PawnOwner->SetActorRotation(UKismetMathLibrary::MakeRotFromXZ(ForwardProjection, PlaneNormal));

            if (MovingPlatform == nullptr)
            {
                // Moves the robot up or down, depending on impact position
                float MinDistance = ContactsDistance[0];
                if (ContactsDistance[1] < MinDistance)
                    MinDistance = ContactsDistance[1];
                if (ContactsDistance[2] < MinDistance)
                    MinDistance = ContactsDistance[2];

                MinDistance -= RayOffsetUp;
                
                if( MinDistance > FallingSpeed * DeltaTime )
                    MinDistance = FallingSpeed * DeltaTime;
                
                FVector HeightVariation = {0., 0., -MinDistance};
                PawnOwner->AddActorWorldOffset(HeightVariation, true, &Hit, ETeleportType::TeleportPhysics);
                // UE_LOG(LogTemp, Warning, TEXT("URobotVehicleMovementComponent::UpdateMovement - Distance Modification = %f"),
                // MinDistance );
            }
        }
    }
}

void URobotVehicleMovementComponent::InitOdom()
{
    OdomData.header_frame_id = FrameId;
    OdomData.child_frame_id = ChildFrameId;

    InitialTransform.SetTranslation(PawnOwner->GetActorLocation());
    InitialTransform.SetRotation(FQuat(PawnOwner->GetActorRotation()));

    LastLocation = InitialTransform.GetTranslation();
	LastOrientation = InitialTransform.GetRotation();

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

void URobotVehicleMovementComponent::UpdateOdom(float DeltaTime)
{
    if (!IsOdomInitialized)
    {
        InitOdom();
    }
    // time
    float TimeNow = UGameplayStatics::GetTimeSeconds(GetWorld());
    OdomData.header_stamp_sec = static_cast<int32>(TimeNow);
    uint64 ns = (uint64)(TimeNow * 1e+09f);
    OdomData.header_stamp_nanosec = static_cast<uint32>(ns - (OdomData.header_stamp_sec * 1e+09));

    // position
    FVector Pos = PawnOwner->GetActorLocation() - InitialTransform.GetTranslation();
    OdomData.pose_pose_position_x = Pos.X;
    OdomData.pose_pose_position_y = Pos.Y;
    OdomData.pose_pose_position_z = Pos.Z;
    OdomData.pose_pose_orientation = FQuat(PawnOwner->GetActorRotation() - InitialTransform.GetRotation().Rotator());

    // compute velocities values with real position and orientation of robot
    FQuat CurrentOrientation = FQuat(PawnOwner->GetActorRotation() - InitialTransform.GetRotation().Rotator());
	FQuat DeltaQ = CurrentOrientation * LastOrientation.Inverse();
    FVector NewAngularVelocity = FMath::DegreesToRadians( DeltaQ.Euler() ) / DeltaTime;

    // linear velocity is expressed in robot local coordinates, taking into account the robot rotation
    FVector NewVelocity = OdomData.pose_pose_orientation.UnrotateVector( Pos-LastLocation ) / DeltaTime;

	LastOrientation = CurrentOrientation;
    LastLocation = Pos;

    // velocity
    OdomData.twist_twist_linear = NewVelocity;
    OdomData.twist_twist_angular = NewAngularVelocity;
}

void URobotVehicleMovementComponent::UpdateEntityState( FString AgentName )
{
    EntityState.name = AgentName + TEXT("/base_footprint");
    EntityState.pose_position_x = OdomData.pose_pose_position_x / 100.f;
    EntityState.pose_position_y = OdomData.pose_pose_position_y / 100.f;
    EntityState.pose_position_z = OdomData.pose_pose_position_z / 100.f;
    EntityState.pose_orientation = OdomData.pose_pose_orientation;
    EntityState.pose_position_y = -EntityState.pose_position_y;
    EntityState.pose_orientation.X = -EntityState.pose_orientation.X;
    EntityState.pose_orientation.Z = -EntityState.pose_orientation.Z;
    EntityState.twist_linear = FVector::ZeroVector;
    EntityState.twist_angular = FVector::ZeroVector;
    EntityState.reference_frame = AgentName + TEXT("/map");
}

void URobotVehicleMovementComponent::TickComponent(float DeltaTime,
                                                   enum ELevelTick TickType,
                                                   FActorComponentTickFunction* ThisTickFunction)
{
    // UE_LOG( LogTemp, Warning, TEXT("URobotVehicleMovementComponent::TickComponent...") );
    if (!ShouldSkipUpdate(DeltaTime))
    {
        Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

        // Make sure that everything is still valid, and that we are allowed to move.
        if (IsValid(UpdatedComponent))
        {
            UpdateMovement(DeltaTime);
            UpdateOdom(DeltaTime);
        }

        UpdateComponentVelocity();
    }
}

FTransform URobotVehicleMovementComponent::GetOdomTF()
{
    FTransform TF;
    FVector Pos(OdomData.pose_pose_position_x, OdomData.pose_pose_position_y, OdomData.pose_pose_position_z);
    TF.SetTranslation(Pos);
    TF.SetRotation(OdomData.pose_pose_orientation);

    return TF;
}

void URobotVehicleMovementComponent::InitMovementComponent()
{
    InitOdom();

    TArray<UActorComponent*> ActorContactPoints = PawnOwner->GetComponentsByTag(USceneComponent::StaticClass(), "ContactPoint");
    for (auto ACP : ActorContactPoints)
    {
        USceneComponent* SCP = Cast<USceneComponent>(ACP);
        if (SCP)
            ContactPoints.Add(SCP);
    }
    UE_LOG(LogTemp,
           Warning,
           TEXT("URobotVehicleMovementComponent::InitMovementComponent - Nb Contact Points : %d"),
           ContactPoints.Num());

    // Compute the starting distance between the robot root and the floor
    // We consider that the robot is on a horizontal floor at the beginning
    FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Contact_Trace")), true, PawnOwner);
    TraceParams.bReturnPhysicalMaterial = true;
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnFaceIndex = true;
    TraceParams.AddIgnoredActor(PawnOwner);

    FVector startPos = PawnOwner->GetActorLocation() + FVector(0., 0., 10.);
    FVector endPos = PawnOwner->GetActorLocation() - FVector(0., 0., 50.);
    FHitResult HitResult;
    bool IsFloorHit = GetWorld()->LineTraceSingleByChannel(HitResult,
                                                           startPos,
                                                           endPos,
                                                           ECollisionChannel::ECC_Visibility,
                                                           TraceParams,
                                                           FCollisionResponseParams::DefaultResponseParam);
    if (IsFloorHit)
    {
        MinDistanceToFloor = HitResult.Distance;
        UE_LOG(LogTemp,
               Warning,
               TEXT("URobotVehicleMovementComponent::InitMovementComponent - Min Distance To Floor = %f"),
               MinDistanceToFloor);
    }
}

void URobotVehicleMovementComponent::SetMovingPlatform(AActor* platform)
{
    // UE_LOG(LogTemp, Warning, TEXT("URobotVehicleMovementComponent::SetMovingPlatform..."));
    MovingPlatform = platform;
    LastPlatformLocation = FVector(platform->GetActorLocation());
    LastPlatformRotation = FQuat(platform->GetActorQuat());
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