// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRAIRobotROSController.h"

// RapyutaSimulationPlugins
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRAIRobotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);
}

void ARRAIRobotROSController::OnUnPossess()
{
    Super::OnUnPossess();
}

void ARRAIRobotROSController::ResetControl()
{
    bRotating = false;
    bLinearMoving = false;
    OnSuccess.Unbind();
    OnFail.Unbind();
}

void ARRAIRobotROSController::SetDelegates(const FMoveCompleteCallback& InOnSuccess,
                                           const FMoveCompleteCallback& InOnFail,
                                           const float InLinearMotionTolerance,
                                           const float InOrientationTolerance,
                                           const float InTimeOut)
{
    if (!InOnSuccess.IsBound())
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("OnSuccess Delegate is not set - is this on purpose? "));
    }
    else
    {
        OnSuccess.Unbind();
        OnSuccess = InOnSuccess;
    }

    if (!InOnFail.IsBound())
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("OnFail Delegate is not set - is this on purpose? "));
    }
    else
    {
        OnFail.Unbind();
        OnFail = InOnFail;
    }

    LinearMotionTolerance = (InLinearMotionTolerance >= 0) ? InLinearMotionTolerance : LinearMotionTolerance;
    OrientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;

    MoveStartTime = GetWorld()->GetTimeSeconds();
    MoveTimeout = InTimeOut;
};

ARRAIRobotROSController* ARRAIRobotROSController::CheckController(APawn* TargetPawn)
{
    if (IsValid(TargetPawn))
    {
        auto controller = Cast<ARRAIRobotROSController>(TargetPawn->GetController());
        if (controller)
        {
            return controller;
        }
        else
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Controller should be child class of ARRAIRobotROSController"));
        }
    }
    else
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("TargetPawn is not valid"));
    }
    return nullptr;
}

EPathFollowingRequestResult::Type ARRAIRobotROSController::MoveToActorWithDelegates(AActor* Goal,
                                                                                    const FMoveCompleteCallback& InOnSuccess,
                                                                                    const FMoveCompleteCallback& InOnFail,
                                                                                    float AcceptanceRadius,
                                                                                    bool bStopOnOverlap,
                                                                                    bool bUsePathfinding,
                                                                                    bool bCanStrafe,
                                                                                    TSubclassOf<UNavigationQueryFilter> FilterClass,
                                                                                    bool bAllowPartialPath,
                                                                                    const float InOrientationTolerance,
                                                                                    const float InTimeOut)
{
    SetDelegates(InOnSuccess, InOnFail, -1, InOrientationTolerance, InTimeOut);
    OrientationTarget = Goal->GetActorRotation();
    AIMovePoseTarget = Goal->GetActorLocation();    // for teleport on fail
    bRotating = false;
    bLinearMoving = false;
    return MoveToActor(Goal, AcceptanceRadius, bStopOnOverlap, bUsePathfinding, bCanStrafe, FilterClass, bAllowPartialPath);
}

EPathFollowingRequestResult::Type ARRAIRobotROSController::MoveToActorWithDelegates(APawn* TargetPawn,
                                                                                    AActor* Goal,
                                                                                    const FMoveCompleteCallback& InOnSuccess,
                                                                                    const FMoveCompleteCallback& InOnFail,
                                                                                    float AcceptanceRadius,
                                                                                    bool bStopOnOverlap,
                                                                                    bool bUsePathfinding,
                                                                                    bool bCanStrafe,
                                                                                    TSubclassOf<UNavigationQueryFilter> FilterClass,
                                                                                    bool bAllowPartialPath,
                                                                                    const float InOrientationTolerance,
                                                                                    const float InTimeOut)
{
    EPathFollowingRequestResult::Type res = EPathFollowingRequestResult::Type::Failed;
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        res = controller->MoveToActorWithDelegates(Goal,
                                                   InOnSuccess,
                                                   InOnFail,
                                                   AcceptanceRadius,
                                                   bStopOnOverlap,
                                                   bUsePathfinding,
                                                   bCanStrafe,
                                                   FilterClass,
                                                   bAllowPartialPath,
                                                   InOrientationTolerance,
                                                   InTimeOut);
    }
    return res;
};

EPathFollowingRequestResult::Type ARRAIRobotROSController::MoveToLocationWithDelegates(
    const FVector& Dest,
    const FRotator& DestRotator,
    const FMoveCompleteCallback& InOnSuccess,
    const FMoveCompleteCallback& InOnFail,
    float AcceptanceRadius,
    bool bStopOnOverlap,
    bool bUsePathfinding,
    bool bProjectDestinationToNavigation,
    bool bCanStrafe,
    TSubclassOf<UNavigationQueryFilter> FilterClass,
    bool bAllowPartialPath,
    const float InOrientationTolerance,
    const float InTimeOut,
    const FVector& InOriginPosition,
    const FRotator& InOriginRotator)
{
    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    FTransform worldDest = URRGeneralUtils::GetWorldTransform(FTransform(InOriginRotator, InOriginPosition, FVector::OneVector),
                                                              FTransform(DestRotator, Dest, FVector::OneVector));
    OrientationTarget = worldDest.GetRotation().Rotator();
    AIMovePoseTarget = worldDest.GetLocation();    // for teleport on fail
    bRotating = false;
    bLinearMoving = false;
    return MoveToLocation(Dest,
                          AcceptanceRadius,
                          bStopOnOverlap,
                          bUsePathfinding,
                          bProjectDestinationToNavigation,
                          bCanStrafe,
                          FilterClass,
                          bAllowPartialPath);
}

EPathFollowingRequestResult::Type ARRAIRobotROSController::MoveToLocationWithDelegates(
    APawn* TargetPawn,
    const FVector& Dest,
    const FRotator& DestRotator,
    const FMoveCompleteCallback& InOnSuccess,
    const FMoveCompleteCallback& InOnFail,
    float AcceptanceRadius,
    bool bStopOnOverlap,
    bool bUsePathfinding,
    bool bProjectDestinationToNavigation,
    bool bCanStrafe,
    TSubclassOf<UNavigationQueryFilter> FilterClass,
    bool bAllowPartialPath,
    const float InOrientationTolerance,
    const float InTimeOut,
    const FVector& InOriginPosition,
    const FRotator& InOriginRotator)
{
    EPathFollowingRequestResult::Type res = EPathFollowingRequestResult::Type::Failed;
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        res = controller->MoveToLocationWithDelegates(Dest,
                                                      DestRotator,
                                                      InOnSuccess,
                                                      InOnFail,
                                                      AcceptanceRadius,
                                                      bStopOnOverlap,
                                                      bUsePathfinding,
                                                      bProjectDestinationToNavigation,
                                                      bCanStrafe,
                                                      FilterClass,
                                                      bAllowPartialPath,
                                                      InOrientationTolerance,
                                                      InTimeOut,
                                                      InOriginPosition,
                                                      InOriginRotator);
    }
    return res;
};

void ARRAIRobotROSController::OnMoveCompleted(FAIRequestID RequestID, const FPathFollowingResult& Result)
{
    Super::OnMoveCompleted(RequestID, Result);
    if (Result.Code == EPathFollowingResult::Success)
    {
        SetOrientationTarget(OrientationTarget, false);
    }
    else
    {
        if (OnFail.IsBound())
        {
            if (bTeleportOnFail)
            {
                GetPawn()->SetActorLocation(AIMovePoseTarget);
                GetPawn()->SetActorRotation(OrientationTarget);
            }
            OnFail.ExecuteIfBound();
        }
    }
}

void ARRAIRobotROSController::LinearMoveToLocationWithDelegates(const FVector& Dest,
                                                                const FRotator& DestRotator,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                float AcceptanceRadius,
                                                                const float InOrientationTolerance,
                                                                const float InTimeOut,
                                                                const FVector& InOriginPosition,
                                                                const FRotator& InOriginRotator)
{
    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    FTransform worldDest = URRGeneralUtils::GetWorldTransform(FTransform(InOriginRotator, InOriginPosition, FVector::OneVector),
                                                              FTransform(DestRotator, Dest, FVector::OneVector));

    bRotating = false;
    bLinearMoving = false;

    FRotator worldDestRotator = worldDest.GetRotation().Rotator();
    FVector worldDestLocation = worldDest.GetLocation();    // for teleport on fail

    // Rotate -> Linear -> Rotate
    OnSuccessInternal.BindLambda(
        [&, worldDestLocation, worldDestRotator]()
        {
            OnSuccessInternal.BindLambda(
                [&, worldDestRotator]()
                {
                    OnSuccessInternal.Unbind();
                    SetOrientationTarget(worldDestRotator, false);
                });
            SetLinearMotionTarget(worldDestLocation, false);
        });

    // Calc orientation to Dest from current ActorLocation
    SetOrientationTarget((worldDestLocation - GetPawn()->GetActorLocation()).Rotation(), false);
}

void ARRAIRobotROSController::LinearMoveToLocationWithDelegates(APawn* TargetPawn,
                                                                const FVector& Dest,
                                                                const FRotator& DestRotator,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                float AcceptanceRadius,
                                                                const float InOrientationTolerance,
                                                                const float InTimeOut,
                                                                const FVector& InOriginPosition,
                                                                const FRotator& InOriginRotator)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->LinearMoveToLocationWithDelegates(Dest,
                                                      DestRotator,
                                                      InOnSuccess,
                                                      InOnFail,
                                                      AcceptanceRadius,
                                                      InOrientationTolerance,
                                                      InTimeOut,
                                                      InOriginPosition,
                                                      InOriginRotator);
    }
}

void ARRAIRobotROSController::SetOrientationTarget(const FRotator& InOrientation,
                                                   const bool InReset,
                                                   const FRotator& InOriginRotator)
{
    OrientationTarget = (InOriginRotator + InOrientation);
    FVector orientationVec = GetPawn()->GetActorRotation().Euler();
    FVector orientationTargetVec = OrientationTarget.Euler();
    for (uint8 i = 0; i < 3; i++)
    {
        float angleDiff = FRotator::NormalizeAxis(orientationTargetVec[i] - orientationVec[i]);
        AngularVelocity[i] = FMath::IsNearlyZero(angleDiff) ? 0 : angleDiff < 0 ? -RotationSpeed : RotationSpeed;
    }

    if (InReset)
    {
        ResetControl();
    }
    bRotating = true;
}

void ARRAIRobotROSController::AddLocalOrientationOffset(const FRotator& InOrientation, const bool InReset)
{
    SetOrientationTarget(InOrientation, InReset, GetPawn()->GetActorRotation());
}

void ARRAIRobotROSController::SetOrientationTarget(APawn* TargetPawn,
                                                   const FRotator& InOrientation,
                                                   const bool InReset,
                                                   const FRotator& InOriginRotator)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetOrientationTarget(InOrientation, InReset, InOriginRotator);
    }
}

void ARRAIRobotROSController::AddLocalOrientationOffset(APawn* TargetPawn, const FRotator& InOrientation, const bool InReset)
{
    SetOrientationTarget(TargetPawn, InOrientation, InReset, TargetPawn->GetActorRotation());
}

void ARRAIRobotROSController::SetOrientationTargetWthDelegates(const FRotator& InOrientation,
                                                               const FMoveCompleteCallback& InOnSuccess,
                                                               const FMoveCompleteCallback& InOnFail,
                                                               const float InOrientationTolerance,
                                                               const float InTimeOut,
                                                               const FRotator& InOriginRotator)
{
    SetDelegates(InOnSuccess, InOnFail, -1, InOrientationTolerance, InTimeOut);
    SetOrientationTarget(InOrientation, false, InOriginRotator);
}

void ARRAIRobotROSController::AddLocalOrientationOffsetWthDelegates(const FRotator& InOrientation,
                                                                    const FMoveCompleteCallback& InOnSuccess,
                                                                    const FMoveCompleteCallback& InOnFail,
                                                                    const float InOrientationTolerance,
                                                                    const float InTimeOut)
{
    SetOrientationTargetWthDelegates(
        InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut, GetPawn()->GetActorRotation());
}

void ARRAIRobotROSController::SetOrientationTargetWthDelegates(APawn* TargetPawn,
                                                               const FRotator& InOrientation,
                                                               const FMoveCompleteCallback& InOnSuccess,
                                                               const FMoveCompleteCallback& InOnFail,
                                                               const float InOrientationTolerance,
                                                               const float InTimeOut,
                                                               const FRotator& InOriginRotator)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetOrientationTargetWthDelegates(
            InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut, InOriginRotator);
    }
}

void ARRAIRobotROSController::AddLocalOrientationOffsetWthDelegates(APawn* TargetPawn,
                                                                    const FRotator& InOrientation,
                                                                    const FMoveCompleteCallback& InOnSuccess,
                                                                    const FMoveCompleteCallback& InOnFail,
                                                                    const float InOrientationTolerance,
                                                                    const float InTimeOut)
{
    SetOrientationTargetWthDelegates(
        TargetPawn, InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut, TargetPawn->GetActorRotation());
}

void ARRAIRobotROSController::SetLinearMotionTarget(const FVector& InPosition,
                                                    const bool InReset,
                                                    const FVector& InOriginPosition,
                                                    const FRotator& InOriginRotator)
{
    FTransform worldDest = URRGeneralUtils::GetWorldTransform(FTransform(InOriginRotator, InOriginPosition, FVector::OneVector),
                                                              FTransform(InPosition));
    LinearMotionTarget = worldDest.GetLocation();

    // FVector location = GetPawn()->GetActorLocation();
    // for (uint8 i = 0; i < 3; i++)
    // {
    //     float diff = LinearMotionTarget[i] - location[i];
    //     LinearVelocity[i] = FMath::IsNearlyZero(diff) ? 0 : diff < 0 ? -LinearSpeed : LinearSpeed;
    // }

    if (InReset)
    {
        ResetControl();
    }
    bLinearMoving = true;
}

void ARRAIRobotROSController::AddLocalLinearMotionOffset(const FVector& InPosition, const bool InReset)
{
    SetLinearMotionTarget(InPosition, InReset, GetPawn()->GetActorLocation(), GetPawn()->GetActorRotation());
}

void ARRAIRobotROSController::SetLinearMotionTarget(APawn* TargetPawn,
                                                    const FVector& InPosition,
                                                    const bool InReset,
                                                    const FVector& InOriginPosition,
                                                    const FRotator& InOriginRotator)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetLinearMotionTarget(InPosition, InReset, InOriginPosition, InOriginRotator);
    }
}

void ARRAIRobotROSController::AddLocalLinearMotionOffset(APawn* TargetPawn, const FVector& InPosition, const bool InReset)
{
    SetLinearMotionTarget(TargetPawn, InPosition, InReset, TargetPawn->GetActorLocation(), TargetPawn->GetActorRotation());
}

void ARRAIRobotROSController::SetLinearMotionTargetWthDelegates(const FVector& InPosition,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                const float InLinearMotionTolerancee,
                                                                const float InTimeOut,
                                                                const FVector& InOriginPosition,
                                                                const FRotator& InOriginRotator)
{
    SetDelegates(InOnSuccess, InOnFail, InLinearMotionTolerancee, -1, InTimeOut);
    SetLinearMotionTarget(InPosition, false, InOriginPosition, InOriginRotator);
}

void ARRAIRobotROSController::AddLocalLinearMotionOffsetWthDelegates(const FVector& InPosition,
                                                                     const FMoveCompleteCallback& InOnSuccess,
                                                                     const FMoveCompleteCallback& InOnFail,
                                                                     const float InLinearMotionTolerancee,
                                                                     const float InTimeOut)
{
    SetLinearMotionTargetWthDelegates(InPosition,
                                      InOnSuccess,
                                      InOnFail,
                                      InLinearMotionTolerancee,
                                      InTimeOut,
                                      GetPawn()->GetActorLocation(),
                                      GetPawn()->GetActorRotation());
}

void ARRAIRobotROSController::SetLinearMotionTargetWthDelegates(APawn* TargetPawn,
                                                                const FVector& InPosition,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                const float InLinearMotionTolerancee,
                                                                const float InTimeOut,
                                                                const FVector& InOriginPosition,
                                                                const FRotator& InOriginRotator)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetLinearMotionTargetWthDelegates(
            InPosition, InOnSuccess, InOnFail, InLinearMotionTolerancee, InTimeOut, InOriginPosition, InOriginRotator);
    }
}
void ARRAIRobotROSController::AddLocalLinearMotionOffsetWthDelegates(APawn* TargetPawn,
                                                                     const FVector& InPosition,
                                                                     const FMoveCompleteCallback& InOnSuccess,
                                                                     const FMoveCompleteCallback& InOnFail,
                                                                     const float InLinearMotionTolerancee,
                                                                     const float InTimeOut)
{
    SetLinearMotionTargetWthDelegates(TargetPawn,
                                      InPosition,
                                      InOnSuccess,
                                      InOnFail,
                                      InLinearMotionTolerancee,
                                      InTimeOut,
                                      TargetPawn->GetActorLocation(),
                                      TargetPawn->GetActorRotation());
}

void ARRAIRobotROSController::UpdateRotation(float DeltaSeconds)
{
    if (bRotating)
    {
        FVector orientationVec = GetPawn()->GetActorRotation().Euler();
        FVector orientationTargetVec = OrientationTarget.Euler();
        FVector dRot = AngularVelocity * DeltaSeconds;
        for (uint8 i = 0; i < 3; i++)
        {
            if (FMath::Abs(orientationVec[i] - orientationTargetVec[i]) < FMath::Abs(OrientationTolerance))
            {
                orientationVec[i] = orientationTargetVec[i];
                AngularVelocity[i] = 0;
            }
            else
            {
                orientationVec[i] += dRot[i];
            }
        }
        FRotator Orientation = FRotator::MakeFromEuler(orientationVec);
        GetPawn()->SetActorRotation(Orientation);
    }
}

void ARRAIRobotROSController::UpdateLinearMotion(float DeltaSeconds)
{
    if (bLinearMoving)
    {
        FVector dPos = LinearMotionTarget - GetPawn()->GetActorLocation();
        if (dPos.Size() <= LinearMotionTolerance)
        {
            GetPawn()->SetActorLocation(LinearMotionTarget);
        }
        else
        {
            GetPawn()->AddActorWorldOffset(LinearSpeed * DeltaSeconds * dPos / dPos.Size());
        }
    }
}

bool ARRAIRobotROSController::HasReachedOrientationTarget(const float InOrientationTolerance)
{
    if (!bRotating)
    {
        return true;
    }

    const float orientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;
    bool res = OrientationTarget.Equals(GetPawn()->GetActorRotation(), orientationTolerance);
    if (res)
    {
        bRotating = false;
        if (OnSuccessInternal.IsBound())
        {
            OnSuccessInternal.ExecuteIfBound();
        }
        else if (OnSuccess.IsBound())
        {
            OnSuccess.ExecuteIfBound();
        }
    }
    else
    {
        // if target is set from MoveToActorWithDelegates
        if (OnFail.IsBound())
        {
            float currentTime = GetWorld()->GetTimeSeconds();
            if (MoveTimeout > 0 && currentTime - MoveStartTime > MoveTimeout)
            {
                if (bTeleportOnFail)
                {
                    GetPawn()->SetActorRotation(OrientationTarget);
                }
                bRotating = false;
                OnFail.ExecuteIfBound();
            }
        }
    }
    return res;
};

bool ARRAIRobotROSController::HasReachedLinearMotionTarget(const float InLinearMotionTolerance)
{
    if (!bLinearMoving)
    {
        return true;
    }

    const float linearMotionTolerance = (InLinearMotionTolerance >= 0) ? InLinearMotionTolerance : LinearMotionTolerance;
    bool res = LinearMotionTarget.Equals(GetPawn()->GetActorLocation(), linearMotionTolerance);

    if (res)
    {
        bLinearMoving = false;
        if (OnSuccessInternal.IsBound())
        {
            OnSuccessInternal.ExecuteIfBound();
        }
        else if (OnSuccess.IsBound())
        {
            OnSuccess.ExecuteIfBound();
        }
    }
    else
    {
        // if target is set from MoveToActorWithDelegates
        if (OnFail.IsBound())
        {
            float currentTime = GetWorld()->GetTimeSeconds();
            if (MoveTimeout > 0 && currentTime - MoveStartTime > MoveTimeout)
            {
                if (bTeleportOnFail)
                {
                    GetPawn()->SetActorLocation(LinearMotionTarget);
                }
                bLinearMoving = false;
                OnFail.ExecuteIfBound();
            }
        }
    }
    return res;
};

void ARRAIRobotROSController::UpdateControl(float DeltaSeconds)
{
    UpdateRotation(DeltaSeconds);
    UpdateLinearMotion(DeltaSeconds);
}

void ARRAIRobotROSController::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (GetPawn())
    {
        UpdateControl(DeltaSeconds);
        HasReachedOrientationTarget();
        HasReachedLinearMotionTarget();
    }
};
