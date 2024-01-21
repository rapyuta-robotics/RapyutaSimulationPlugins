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

void ARRAIRobotROSController::SetDelegates(const FMoveCompleteCallback& InOnSuccess,
                                           const FMoveCompleteCallback& InOnFail,
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

    OrientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;

    MoveStartTime = GetWorld()->GetTimeSeconds();
    MoveTimeout = InTimeOut;
};

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
    SetDelegates(InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut);
    OrientationTarget = Goal->GetActorRotation();
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
    if (IsValid(TargetPawn))
    {
        auto controller = Cast<ARRAIRobotROSController>(TargetPawn->GetController());
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
        else
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Controller should be child class of ARRAIRobotROSController"));
        }
    }
    else
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("TargetPawn is not valid"));
    }
    return res;
};

void ARRAIRobotROSController::OnMoveCompleted(FAIRequestID RequestID, const FPathFollowingResult& Result)
{
    Super::OnMoveCompleted(RequestID, Result);
    if (Result.Code == EPathFollowingResult::Success)
    {
        SetOrientationTarget(OrientationTarget);
    }
    else
    {
        if (OnFail.IsBound())
        {
            OnFail.ExecuteIfBound();
            OnFail.Unbind();
        }
    }
}

void ARRAIRobotROSController::SetOrientationTarget(const FRotator& InOrientation, const bool InStartMoving = false)
{
    OrientationTarget = InOrientation;
    FVector orientationVec = GetPawn()->GetActorRotation().Euler();
    FVector orientationTargetVec = OrientationTarget.Euler();
    for (uint8 i = 0; i < 3; i++)
    {
        float angleDiff = FRotator::NormalizeAxis(orientationTargetVec[i] - orientationVec[i]);
        AngularVelocity[i] = FMath::IsNearlyZero(angleDiff) ? 0 : angleDiff < 0 ? -RotationSpeed : RotationSpeed;
    }
    bRotating = InStartMoving;
    OnSuccess.Unbind();
    OnFail.Unbind();
    MoveTimeout = -1.0f;
}

void ARRAIRobotROSController::SetLinearMotionTarget(const FRotator& InPosition, const bool InStartMoving = true)
{
    FVector location = GetPawn()->GetActorLocation();
    LinearMotionTarget = InPosition;
    for (uint8 i = 0; i < 3; i++)
    {
        float diff = LinearMotionTarget[i] - location[i];
        LinearVelocity[i] = FMath::IsNearlyZero(diff) ? 0 : diff < 0 ? -LinearSpeed : LinearSpeed;
    }
    bLinearMoving = InStartMoving;
    OnSuccess.Unbind();
    OnFail.Unbind();
    MoveTimeout = -1.0f;
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
        FVector dPos = LinearVelocity * DeltaSeconds;
        for (uint8 i = 0; i < 3; i++)
        {
            if (FMath::Abs(location[i] - LinearMotionTarget[i]) < FMath::Abs(LinearMotionTolerance))
            {
                location[i] = LinearMotionTarget[i];
                LinearVelocity[i] = 0;
            }
            else
            {
                location[i] += dPos[i];
            }
        }
        GetPawn()->SetActorLocation(location);
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
        if (OnSuccess.IsBound())
        {
            OnSuccess.ExecuteIfBound();
            OnSuccess.Unbind();
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
                bRotating = false;
                OnFail.ExecuteIfBound();
                OnFail.Unbind();
            }
        }
    }
    return res;
};

bool ARRAIRobotROSController::HasReachedLinearMotionTarget(const float InLinearMotionTolerance)
{
    if (!bRotating)
    {
        return true;
    }

    const float linearMotionTolerance = (InLinearMotionTolerance >= 0) ? InLinearMotionTolerance : LinearMotionTolerance;
    bool res = LinearMotionTarget.Equals(GetPawn()->GetActorLocation(), linearMotionTolerance);

    if (res)
    {
        bLinearMoving = false;
        if (OnSuccess.IsBound())
        {
            OnSuccess.ExecuteIfBound();
            OnSuccess.Unbind();
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
                bLinearMoving = false;
                OnFail.ExecuteIfBound();
                OnFail.Unbind();
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
