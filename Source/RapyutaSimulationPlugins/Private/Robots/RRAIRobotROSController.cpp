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
    const float InTimeOut)
{
    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    OrientationTarget = DestRotator;
    AIMovePoseTarget = Dest;    // for teleport on fail
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
    const float InTimeOut)
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
                                                      InTimeOut);
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
                                                                const float InTimeOut)
{
    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    bRotating = false;
    bLinearMoving = false;

    FRotator origDestRotator = DestRotator;
    // Rotate -> Linear -> Rotate
    OnSuccessInternal.BindLambda(
        [&, origDestRotator]()
        {
            OnSuccessInternal.BindLambda(
                [&, origDestRotator]()
                {
                    OnSuccessInternal.Unbind();
                    SetOrientationTarget(origDestRotator, false);
                });
            SetLinearMotionTarget(Dest, false);
        });

    // Calc orientation to Dest from current ActorLocation
    SetOrientationTarget((Dest - GetPawn()->GetActorLocation()).Rotation(), false);
}

void ARRAIRobotROSController::LinearMoveToLocationWithDelegates(APawn* TargetPawn,
                                                                const FVector& Dest,
                                                                const FRotator& DestRotator,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                float AcceptanceRadius,
                                                                const float InOrientationTolerance,
                                                                const float InTimeOut)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->LinearMoveToLocationWithDelegates(
            Dest, DestRotator, InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    }
}

void ARRAIRobotROSController::SetOrientationTarget(const FRotator& InOrientation, const bool InReset)
{
    OrientationTarget = InOrientation;
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

void ARRAIRobotROSController::SetRelativeOrientationTarget(const FRotator& InOrientation, const bool InReset)
{
    SetOrientationTarget(GetPawn()->GetActorRotation() + InOrientation, InReset);
}

void ARRAIRobotROSController::SetOrientationTarget(APawn* TargetPawn, const FRotator& InOrientation)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetOrientationTarget(InOrientation);
    }
}

void ARRAIRobotROSController::SetRelativeOrientationTarget(APawn* TargetPawn, const FRotator& InOrientation)
{
    SetOrientationTarget(TargetPawn, TargetPawn->GetActorRotation() + InOrientation);
}

void ARRAIRobotROSController::SetOrientationTargetWthDelegates(const FRotator& InOrientation,
                                                               const FMoveCompleteCallback& InOnSuccess,
                                                               const FMoveCompleteCallback& InOnFail,
                                                               const float InOrientationTolerance,
                                                               const float InTimeOut)
{
    SetDelegates(InOnSuccess, InOnFail, -1, InOrientationTolerance, InTimeOut);
    SetOrientationTarget(InOrientation, false);
}

void ARRAIRobotROSController::SetRelativeOrientationTargetWthDelegates(const FRotator& InOrientation,
                                                                       const FMoveCompleteCallback& InOnSuccess,
                                                                       const FMoveCompleteCallback& InOnFail,
                                                                       const float InOrientationTolerance,
                                                                       const float InTimeOut)
{
    SetOrientationTargetWthDelegates(
        GetPawn()->GetActorRotation() + InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut);
}

void ARRAIRobotROSController::SetOrientationTargetWthDelegates(APawn* TargetPawn,
                                                               const FRotator& InOrientation,
                                                               const FMoveCompleteCallback& InOnSuccess,
                                                               const FMoveCompleteCallback& InOnFail,
                                                               const float InOrientationTolerance,
                                                               const float InTimeOut)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetOrientationTargetWthDelegates(InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut);
    }
}

void ARRAIRobotROSController::SetRelativeOrientationTargetWthDelegates(APawn* TargetPawn,
                                                                       const FRotator& InOrientation,
                                                                       const FMoveCompleteCallback& InOnSuccess,
                                                                       const FMoveCompleteCallback& InOnFail,
                                                                       const float InOrientationTolerance,
                                                                       const float InTimeOut)
{
    SetOrientationTargetWthDelegates(
        TargetPawn, TargetPawn->GetActorRotation() + InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut);
}

void ARRAIRobotROSController::SetLinearMotionTarget(const FVector& InPosition, const bool InReset)
{
    FVector location = GetPawn()->GetActorLocation();
    LinearMotionTarget = InPosition;
    for (uint8 i = 0; i < 3; i++)
    {
        float diff = LinearMotionTarget[i] - location[i];
        LinearVelocity[i] = FMath::IsNearlyZero(diff) ? 0 : diff < 0 ? -LinearSpeed : LinearSpeed;
    }

    if (InReset)
    {
        ResetControl();
    }
    bLinearMoving = true;
}

void ARRAIRobotROSController::SetRelativeLinearMotionTarget(const FVector& InPosition, const bool InReset)
{
    FVector position = URRGeneralUtils::GetWorldTransform(GetPawn(), FTransform(InPosition)).GetLocation();
    SetLinearMotionTarget(position, InReset);
}

void ARRAIRobotROSController::SetLinearMotionTarget(APawn* TargetPawn, const FVector& InPosition)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetLinearMotionTarget(InPosition);
    }
}

void ARRAIRobotROSController::SetRelativeLinearMotionTarget(APawn* TargetPawn, const FVector& InPosition)
{
    FVector position = URRGeneralUtils::GetWorldTransform(TargetPawn, FTransform(InPosition)).GetLocation();
    SetLinearMotionTarget(TargetPawn, position);
}

void ARRAIRobotROSController::SetLinearMotionTargetWthDelegates(const FVector& InPosition,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                const float InLinearMotionTolerancee,
                                                                const float InTimeOut)
{
    SetDelegates(InOnSuccess, InOnFail, InLinearMotionTolerancee, -1, InTimeOut);
    SetLinearMotionTarget(InPosition, false);
}

void ARRAIRobotROSController::SetRelativeLinearMotionTargetWthDelegates(const FVector& InPosition,
                                                                        const FMoveCompleteCallback& InOnSuccess,
                                                                        const FMoveCompleteCallback& InOnFail,
                                                                        const float InLinearMotionTolerancee,
                                                                        const float InTimeOut)
{
    FVector position = URRGeneralUtils::GetWorldTransform(GetPawn(), FTransform(InPosition)).GetLocation();
    SetLinearMotionTargetWthDelegates(position, InOnSuccess, InOnFail, InLinearMotionTolerancee, InTimeOut);
}

void ARRAIRobotROSController::SetLinearMotionTargetWthDelegates(APawn* TargetPawn,
                                                                const FVector& InPosition,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                const float InLinearMotionTolerancee,
                                                                const float InTimeOut)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        controller->SetLinearMotionTargetWthDelegates(InPosition, InOnSuccess, InOnFail, InLinearMotionTolerancee, InTimeOut);
    }
}
void ARRAIRobotROSController::SetRelativeLinearMotionTargetWthDelegates(APawn* TargetPawn,
                                                                        const FVector& InPosition,
                                                                        const FMoveCompleteCallback& InOnSuccess,
                                                                        const FMoveCompleteCallback& InOnFail,
                                                                        const float InLinearMotionTolerancee,
                                                                        const float InTimeOut)
{
    FVector position = URRGeneralUtils::GetWorldTransform(TargetPawn, FTransform(InPosition)).GetLocation();
    SetLinearMotionTargetWthDelegates(TargetPawn, position, InOnSuccess, InOnFail, InLinearMotionTolerancee, InTimeOut);
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
        FVector location = GetPawn()->GetActorLocation();
        // FVector dPos = LinearVelocity * DeltaSeconds;
        // for (uint8 i = 0; i < 3; i++)
        // {
        //     if (FMath::Abs(location[i] - LinearMotionTarget[i]) < FMath::Abs(LinearMotionTolerance))
        //     {
        //         location[i] = LinearMotionTarget[i];
        //         LinearVelocity[i] = 0;
        //     }
        //     else
        //     {
        //         location[i] += dPos[i];
        //     }
        // }
        // GetPawn()->SetActorLocation(location);
        FVector dPos = LinearMotionTarget - location;
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
