/// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRAIRobotROSController.h"

// rclUE
#include "Msgs/ROS2Int32.h"
#include "Msgs/ROS2PoseStamped.h"
#include "Msgs/ROS2Str.h"
#include "ROS2NodeComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"
#include "Robots/RRBaseRobot.h"
#include "Robots/RRRobotROS2Interface.h"

void ARRAIRobotROSController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    // add ros interface
    // 1. subscribe pose goal
    // 2. subscribe string name goal
    // 3. pub navigation status
    // 4. subscribe param,, set_vel, set_angular_vel
    // 5. Parse parameter, topic name, initial params
    if (InPawn)
    {
        // ROS2_CREATE_SUBSCRIBER(ROS2Interface->RobotROS2Node,
        //                        this,
        //                        PoseGoalTopicName,
        //                        UROS2StrMsg::StaticClass(),
        //                        &ARRAIRobotROSController::PoseGoalCallback);
        // ROS2_CREATE_SUBSCRIBER(ROS2Interface->RobotROS2Node,
        //                        this,
        //                        ActorGoalTopicName,
        //                        UROS2StrMsg::StaticClass(),
        //                        &ARRAIRobotROSController::ActorGoalCallback);
        // ROS2_CREATE_LOOP_PUBLISHER(ROS2Interface->RobotROS2Node,
        //                            this,
        //                            NavStatusTopicName,
        //                            UROS2Publisher::StaticClass(),
        //                            UROS2Int32Msg::StaticClass(),
        //                            NavStatusPublicationFrequencyHz,
        //                            &ARRAIRobotROSController::UpdateNavStatus,
        //                            NavStatusPublisher);
    }
}

void ARRAIRobotROSController::OnUnPossess()
{
    Super::OnUnPossess();
}

void ARRAIRobotROSController::UpdateNavStatus(UROS2GenericMsg* InMessage)
{
    FROSInt32 msg;
    msg.Data = (int)NavStatus;
    CastChecked<UROS2Int32Msg>(InMessage)->SetMsg(msg);
}

void ARRAIRobotROSController::PoseGoalCallback(const UROS2GenericMsg* Msg)
{
    const UROS2PoseStampedMsg* poseStampedMsg = Cast<UROS2PoseStampedMsg>(Msg);
    if (IsValid(poseStampedMsg))
    {
        FROSPoseStamped poseStamped;
        poseStampedMsg->GetMsg(poseStamped);
        // move to  with delegates
        // inside delegate: pub nav status
    }
}

void ARRAIRobotROSController::ActorGoalCallback(const UROS2GenericMsg* Msg)
{
    const UROS2StrMsg* strMsg = Cast<UROS2StrMsg>(Msg);
    if (IsValid(strMsg))
    {
        FROSStr str;
        strMsg->GetMsg(str);

        auto targetActor = URRGeneralUtils::FindActorByName<AActor>(GetWorld(), str.Data);
        if (!targetActor)
        {
            UE_LOG_WITH_INFO_SHORT_NAMED(LogTemp, Warning, TEXT("Can't find actor named: %s"), *str.Data);
            return;
        }

        FMoveCompleteCallback onSuccess, onFail;    // dummy
        MoveToActorWithDelegates(targetActor, onSuccess, onFail);
    }
}

void ARRAIRobotROSController::ResetControl()
{
    NavStatus = ERRAIRobotNavStatus::IDLE;
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
    NavStatus = ERRAIRobotNavStatus::AI_MOVING;
    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    OrientationTarget = Goal->GetActorRotation();
    AIMovePoseTarget = Goal->GetActorLocation();    // for teleport on fail

    auto res = MoveToActor(Goal, AcceptanceRadius, bStopOnOverlap, bUsePathfinding, bCanStrafe, FilterClass, bAllowPartialPath);
    if (res == EPathFollowingRequestResult::Type::AlreadyAtGoal)
    {
        if (SetOrientationTarget(OrientationTarget, false))
        {
            res = EPathFollowingRequestResult::Type::RequestSuccessful;
        }
        else
        {
            ResetControl();
        }
    }
    return res;
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
    // todo check already in tolerance
    NavStatus = ERRAIRobotNavStatus::AI_MOVING;
    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    FTransform worldDest = URRGeneralUtils::GetWorldTransform(FTransform(InOriginRotator, InOriginPosition, FVector::OneVector),
                                                              FTransform(DestRotator, Dest, FVector::OneVector));
    OrientationTarget = worldDest.GetRotation().Rotator();
    AIMovePoseTarget = worldDest.GetLocation();    // for teleport on fail

    auto res = MoveToLocation(Dest,
                              AcceptanceRadius,
                              bStopOnOverlap,
                              bUsePathfinding,
                              bProjectDestinationToNavigation,
                              bCanStrafe,
                              FilterClass,
                              bAllowPartialPath);

    if (res == EPathFollowingRequestResult::Type::AlreadyAtGoal)
    {
        if (SetOrientationTarget(OrientationTarget, false))
        {
            res = EPathFollowingRequestResult::Type::RequestSuccessful;
        }
        else
        {
            ResetControl();
        }
    }
    return res;
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
    if (NavStatus == ERRAIRobotNavStatus::AI_MOVING)
    {
        NavStatus = ERRAIRobotNavStatus::IDLE;
        if (Result.Code == EPathFollowingResult::Success)
        {
            SetOrientationTarget(OrientationTarget, false);
        }
        else
        {
            if (bTeleportOnFail)
            {
                GetPawn()->SetActorLocation(AIMovePoseTarget);
                GetPawn()->SetActorRotation(OrientationTarget);
            }
            if (OnFail.IsBound())
            {
                OnFail.ExecuteIfBound();
            }
        }
    }
}

bool ARRAIRobotROSController::LinearMoveToLocationWithDelegates(const FVector& Dest,
                                                                const FRotator& DestRotator,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                float AcceptanceRadius,
                                                                const float InOrientationTolerance,
                                                                const float InTimeOut,
                                                                const FVector& InOriginPosition,
                                                                const FRotator& InOriginRotator)
{
    FTransform worldDest = URRGeneralUtils::GetWorldTransform(FTransform(InOriginRotator, InOriginPosition, FVector::OneVector),
                                                              FTransform(DestRotator, Dest, FVector::OneVector));

    FRotator worldDestRotator = worldDest.GetRotation().Rotator();
    FVector worldDestLocation = worldDest.GetLocation();    // for teleport on fail

    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    if (HasReachedLinearMotionTarget(worldDestLocation))
    {
        return SetOrientationTarget(worldDestRotator, false);
    }

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

    return SetOrientationTarget((worldDestLocation - GetPawn()->GetActorLocation()).Rotation(), false);
}

bool ARRAIRobotROSController::LinearMoveToLocationWithDelegates(APawn* TargetPawn,
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
        return controller->LinearMoveToLocationWithDelegates(Dest,
                                                             DestRotator,
                                                             InOnSuccess,
                                                             InOnFail,
                                                             AcceptanceRadius,
                                                             InOrientationTolerance,
                                                             InTimeOut,
                                                             InOriginPosition,
                                                             InOriginRotator);
    }
    else
    {
        return false;
    }
}

bool ARRAIRobotROSController::SetOrientationTarget(const FRotator& InOrientation,
                                                   const bool InReset,
                                                   const FRotator& InOriginRotator)
{
    NavStatus = ERRAIRobotNavStatus::ROTATING;
    OrientationTarget = (InOriginRotator + InOrientation);
    if (HasReachedOrientationTarget())
    {
        UE_LOG_WITH_INFO_SHORT(LogTemp, Log, TEXT("Already at goal orientation"));
        return false;
    }

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

    return true;
}

bool ARRAIRobotROSController::AddLocalOrientationOffset(const FRotator& InOrientation, const bool InReset)
{
    return SetOrientationTarget(InOrientation, InReset, GetPawn()->GetActorRotation());
}

bool ARRAIRobotROSController::SetOrientationTarget(APawn* TargetPawn,
                                                   const FRotator& InOrientation,
                                                   const bool InReset,
                                                   const FRotator& InOriginRotator)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        return controller->SetOrientationTarget(InOrientation, InReset, InOriginRotator);
    }
    else
    {
        return false;
    }
}

bool ARRAIRobotROSController::AddLocalOrientationOffset(APawn* TargetPawn, const FRotator& InOrientation, const bool InReset)
{
    return SetOrientationTarget(TargetPawn, InOrientation, InReset, TargetPawn->GetActorRotation());
}

bool ARRAIRobotROSController::SetOrientationTargetWthDelegates(const FRotator& InOrientation,
                                                               const FMoveCompleteCallback& InOnSuccess,
                                                               const FMoveCompleteCallback& InOnFail,
                                                               const float InOrientationTolerance,
                                                               const float InTimeOut,
                                                               const FRotator& InOriginRotator)
{
    SetDelegates(InOnSuccess, InOnFail, -1, InOrientationTolerance, InTimeOut);
    return SetOrientationTarget(InOrientation, false, InOriginRotator);
}

bool ARRAIRobotROSController::AddLocalOrientationOffsetWthDelegates(const FRotator& InOrientation,
                                                                    const FMoveCompleteCallback& InOnSuccess,
                                                                    const FMoveCompleteCallback& InOnFail,
                                                                    const float InOrientationTolerance,
                                                                    const float InTimeOut)
{
    return SetOrientationTargetWthDelegates(
        InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut, GetPawn()->GetActorRotation());
}

bool ARRAIRobotROSController::SetOrientationTargetWthDelegates(APawn* TargetPawn,
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
        return controller->SetOrientationTargetWthDelegates(
            InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut, InOriginRotator);
    }
    else
    {
        return false;
    }
}

bool ARRAIRobotROSController::AddLocalOrientationOffsetWthDelegates(APawn* TargetPawn,
                                                                    const FRotator& InOrientation,
                                                                    const FMoveCompleteCallback& InOnSuccess,
                                                                    const FMoveCompleteCallback& InOnFail,
                                                                    const float InOrientationTolerance,
                                                                    const float InTimeOut)
{
    return SetOrientationTargetWthDelegates(
        TargetPawn, InOrientation, InOnSuccess, InOnFail, InOrientationTolerance, InTimeOut, TargetPawn->GetActorRotation());
}

bool ARRAIRobotROSController::SetLinearMotionTarget(const FVector& InPosition,
                                                    const bool InReset,
                                                    const FVector& InOriginPosition,
                                                    const FRotator& InOriginRotator)
{
    FTransform worldDest = URRGeneralUtils::GetWorldTransform(FTransform(InOriginRotator, InOriginPosition, FVector::OneVector),
                                                              FTransform(InPosition));

    LinearMotionTarget = worldDest.GetLocation();
    NavStatus = ERRAIRobotNavStatus::LINEAR_MOVING;
    if (HasReachedLinearMotionTarget())
    {
        UE_LOG_WITH_INFO_SHORT(LogTemp, Log, TEXT("Already at goal location"));
        return false;
    }

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

    return true;
}

bool ARRAIRobotROSController::AddLocalLinearMotionOffset(const FVector& InPosition, const bool InReset)
{
    return SetLinearMotionTarget(InPosition, InReset, GetPawn()->GetActorLocation(), GetPawn()->GetActorRotation());
}

bool ARRAIRobotROSController::SetLinearMotionTarget(APawn* TargetPawn,
                                                    const FVector& InPosition,
                                                    const bool InReset,
                                                    const FVector& InOriginPosition,
                                                    const FRotator& InOriginRotator)
{
    auto controller = CheckController(TargetPawn);
    if (controller)
    {
        return controller->SetLinearMotionTarget(InPosition, InReset, InOriginPosition, InOriginRotator);
    }
    else
    {
        return false;
    }
}

bool ARRAIRobotROSController::AddLocalLinearMotionOffset(APawn* TargetPawn, const FVector& InPosition, const bool InReset)
{
    return SetLinearMotionTarget(TargetPawn, InPosition, InReset, TargetPawn->GetActorLocation(), TargetPawn->GetActorRotation());
}

bool ARRAIRobotROSController::SetLinearMotionTargetWthDelegates(const FVector& InPosition,
                                                                const FMoveCompleteCallback& InOnSuccess,
                                                                const FMoveCompleteCallback& InOnFail,
                                                                const float InLinearMotionTolerancee,
                                                                const float InTimeOut,
                                                                const FVector& InOriginPosition,
                                                                const FRotator& InOriginRotator)
{
    SetDelegates(InOnSuccess, InOnFail, InLinearMotionTolerancee, -1, InTimeOut);
    return SetLinearMotionTarget(InPosition, false, InOriginPosition, InOriginRotator);
}

bool ARRAIRobotROSController::AddLocalLinearMotionOffsetWthDelegates(const FVector& InPosition,
                                                                     const FMoveCompleteCallback& InOnSuccess,
                                                                     const FMoveCompleteCallback& InOnFail,
                                                                     const float InLinearMotionTolerancee,
                                                                     const float InTimeOut)
{
    return SetLinearMotionTargetWthDelegates(InPosition,
                                             InOnSuccess,
                                             InOnFail,
                                             InLinearMotionTolerancee,
                                             InTimeOut,
                                             GetPawn()->GetActorLocation(),
                                             GetPawn()->GetActorRotation());
}

bool ARRAIRobotROSController::SetLinearMotionTargetWthDelegates(APawn* TargetPawn,
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
        return controller->SetLinearMotionTargetWthDelegates(
            InPosition, InOnSuccess, InOnFail, InLinearMotionTolerancee, InTimeOut, InOriginPosition, InOriginRotator);
    }
    else
    {
        return false;
    }
}
bool ARRAIRobotROSController::AddLocalLinearMotionOffsetWthDelegates(APawn* TargetPawn,
                                                                     const FVector& InPosition,
                                                                     const FMoveCompleteCallback& InOnSuccess,
                                                                     const FMoveCompleteCallback& InOnFail,
                                                                     const float InLinearMotionTolerancee,
                                                                     const float InTimeOut)
{
    return SetLinearMotionTargetWthDelegates(TargetPawn,
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
    if (NavStatus == ERRAIRobotNavStatus::ROTATING)
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
    if (NavStatus == ERRAIRobotNavStatus::LINEAR_MOVING)
    {
        UE_LOG_WITH_INFO_SHORT(
            LogTemp, Log, TEXT("%s, %s"), *LinearMotionTarget.ToString(), *GetPawn()->GetActorLocation().ToString());
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

bool ARRAIRobotROSController::HasReachedOrientationTarget(const FRotator InOrientationTarget, const float InOrientationTolerance)
{
    const float orientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;
    return OrientationTarget.Equals(GetPawn()->GetActorRotation(), orientationTolerance);
}

bool ARRAIRobotROSController::HasReachedOrientationTarget(const float InOrientationTolerance)
{
    if (NavStatus != ERRAIRobotNavStatus::ROTATING)
    {
        return true;
    }

    bool res = HasReachedOrientationTarget(OrientationTarget, InOrientationTolerance);
    if (res)
    {
        NavStatus = ERRAIRobotNavStatus::IDLE;
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
                NavStatus = ERRAIRobotNavStatus::IDLE;
                OnFail.ExecuteIfBound();
            }
        }
    }
    return res;
};

bool ARRAIRobotROSController::HasReachedLinearMotionTarget(const FVector InLinearMotionTarget, const float InLinearMotionTolerance)
{
    const float linearMotionTolerance = (InLinearMotionTolerance >= 0) ? InLinearMotionTolerance : LinearMotionTolerance;
    return InLinearMotionTarget.Equals(GetPawn()->GetActorLocation(), linearMotionTolerance);
}

bool ARRAIRobotROSController::HasReachedLinearMotionTarget(const float InLinearMotionTolerance)
{
    if (NavStatus != ERRAIRobotNavStatus::LINEAR_MOVING)
    {
        return true;
    }

    bool res = HasReachedLinearMotionTarget(LinearMotionTarget, InLinearMotionTolerance);
    if (res)
    {
        NavStatus = ERRAIRobotNavStatus::IDLE;
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
                NavStatus = ERRAIRobotNavStatus::IDLE;
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
