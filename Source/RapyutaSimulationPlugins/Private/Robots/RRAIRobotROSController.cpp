/// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RRAIRobotROSController.h"

// rclUE
#include "Msgs/ROS2Float32.h"
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

    /**
     * @todo this won't work with client-server
    */
    InitROS2Interface();
}

void ARRAIRobotROSController::OnUnPossess()
{
    Super::OnUnPossess();
}

void ARRAIRobotROSController::InitROS2Interface()
{
    InitPropertiesFromJSON();
    if (!InitROS2InterfaceImpl())
    {
        GetWorld()->GetTimerManager().SetTimer(
            ROS2InitTimer, FTimerDelegate::CreateLambda([this] { InitROS2InterfaceImpl(); }), 1.0f, true);
    }
}

bool ARRAIRobotROSController::InitPropertiesFromJSON()
{
    // Verify [ROSSpawnParameters], which houses JSON
    if (nullptr == ROS2Interface || nullptr == ROS2Interface->ROSSpawnParameters)
    {
        return false;
    }

    TSharedRef<TJsonReader<TCHAR>> jsonReader =
        TJsonReaderFactory<TCHAR>::Create(ROS2Interface->ROSSpawnParameters->ActorJsonConfigs);
    TSharedPtr<FJsonObject> jsonObj = MakeShareable(new FJsonObject());
    if (!FJsonSerializer::Deserialize(jsonReader, jsonObj) && jsonObj.IsValid())
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Error, TEXT("Failed to deserialize json to object"));
        return false;
    }

    // Parse single value
    bool bParam = false;
    int intParam = 0;
    float floatParam = 0.f;
    FTransform transformParam = FTransform::Identity;
    FVector vectorParam = FVector::ZeroVector;
    FString stringParam = TEXT("");

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("debug"), bParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("debug value: %d"), bParam);
        bDebug = bParam;
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("mode"), intParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("mode value: %d"), intParam);
        if (intParam > (int)ERRAIRobotMode::BEGIN && intParam < (int)ERRAIRobotMode::END)
        {
            Mode = static_cast<ERRAIRobotMode>(intParam);
        }
        else
        {
            Mode = ERRAIRobotMode::MANUAL;
        }
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("speed"), floatParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("speed value: %f"), floatParam);
        SetSpeed(URRConversionUtils::DistanceROSToUE(floatParam));
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("angular_vel"), floatParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("angular_vel value: %f"), floatParam);
        SetRotationRate(FMath::RadiansToDegrees(floatParam));
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("acceleration"), floatParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("acceleration value: %f"), floatParam);
        SetAcceleration(URRConversionUtils::DistanceROSToUE(floatParam));
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("acceptance_radius"), floatParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("acceptance_radius value: %f"), floatParam);
        const float InLinearMotionTolerance = URRConversionUtils::DistanceROSToUE(floatParam);
        LinearMotionTolerance = (InLinearMotionTolerance >= 0) ? InLinearMotionTolerance : LinearMotionTolerance;
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("orientation_tolerance"), floatParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("orientation_tolerance value: %f"), floatParam);
        const float InOrientationTolerance = FMath::RadiansToDegrees(floatParam);
        OrientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("random_move_bounding_box"), vectorParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("random_move_bounding_box value: %s"), *vectorParam.ToString());
        RandomMoveBoundingBox = URRConversionUtils::VectorROSToUE(vectorParam);
    }

    if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("origin"), vectorParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("origin value: %s"), *vectorParam.ToString());
        RandomMoveBoundingBox = URRConversionUtils::VectorROSToUE(vectorParam);
    }
    else if (URRGeneralUtils::GetJsonField(jsonObj, TEXT("origin"), stringParam))
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("origin value: %s"), *stringParam);
        OriginActor = URRGeneralUtils::FindActorByName<AActor>(GetWorld(), stringParam);
        if (!OriginActor)
        {
            UE_LOG_WITH_INFO_SHORT_NAMED(LogTemp, Warning, TEXT("Can't find actor named: %s"), *stringParam);
        }
    }

    const TArray<TSharedPtr<FJsonValue>>* paramArray;
    if (jsonObj->TryGetArrayField(TEXT("goal_sequence"), paramArray))
    {
        for (const auto& param : *paramArray)
        {
            if (URRGeneralUtils::GetJsonField(param->AsObject(), TEXT("pose"), transformParam))
            {
                FTransform ROStf = URRConversionUtils::TransformROSToUE(transformParam);
                UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("goal_sequence pose value: %s"), *ROStf.ToString());
                GoalSequence.Add(ROStf);
            }
            else if (URRGeneralUtils::GetJsonField(param->AsObject(), TEXT("name"), stringParam))
            {
                auto targetActor = URRGeneralUtils::FindActorByName<AActor>(GetWorld(), stringParam);
                if (!targetActor)
                {
                    UE_LOG_WITH_INFO_SHORT_NAMED(LogTemp, Warning, TEXT("Can't find actor named: %s"), *stringParam);
                }
                else
                {
                    UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("goal_sequence name value: %s"), *stringParam);
                    GoalSequence.Add(targetActor->GetTransform());
                }
            }
        }
    }

    return true;
}

bool ARRAIRobotROSController::SetSpeed(const float InSpeed)
{
    LinearSpeed = InSpeed;
    if (GetPawn() && GetPawn()->GetMovementComponent())
    {
        auto floatingMoveComp = Cast<UFloatingPawnMovement>(GetPawn()->GetMovementComponent());
        if (floatingMoveComp)
        {
            floatingMoveComp->MaxSpeed = InSpeed;
            return true;
        }

        auto characterMoveComp = Cast<UCharacterMovementComponent>(GetPawn()->GetMovementComponent());
        if (characterMoveComp)
        {
            characterMoveComp->MaxWalkSpeed = characterMoveComp->MaxCustomMovementSpeed = characterMoveComp->MaxFlySpeed =
                characterMoveComp->MaxSwimSpeed = InSpeed;
            return true;
        }

        UE_LOG_WITH_INFO_NAMED(
            LogRapyutaCore,
            Warning,
            TEXT("MovementComponent must be child class of UFloatingPawnMovement or UCharacterMovementComponent."));
        return false;
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("Pawn or MovementComponent is nullptr"));
        return false;
    }
}

bool ARRAIRobotROSController::SetRotationRate(const float InRotationRate)
{
    RotationRate = InRotationRate;
    if (GetPawn() && GetPawn()->GetMovementComponent())
    {
        auto characterMoveComp = Cast<UCharacterMovementComponent>(GetPawn()->GetMovementComponent());
        if (characterMoveComp)
        {
            characterMoveComp->RotationRate = InRotationRate * FRotator(1.0, 1.0, 1.0);
            return true;
        }

        UE_LOG_WITH_INFO_NAMED(
            LogRapyutaCore, Warning, TEXT("MovementComponent must be child class of UCharacterMovementComponent."));
        return false;
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("Pawn or MovementComponent is nullptr"));
        return false;
    }
}

bool ARRAIRobotROSController::SetAcceleration(const float InAcceleration)
{
    if (GetPawn() && GetPawn()->GetMovementComponent())
    {
        auto floatingMoveComp = Cast<UFloatingPawnMovement>(GetPawn()->GetMovementComponent());
        if (floatingMoveComp)
        {
            floatingMoveComp->Acceleration = floatingMoveComp->Deceleration = InAcceleration;
            return true;
        }

        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("MovementComponent must be child class of UFloatingPawnMovement."));
        return false;
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("Pawn or MovementComponent is nullptr"));
        return false;
    }
}

bool ARRAIRobotROSController::InitROS2InterfaceImpl()
{
    // add ros interface
    // 1. subscribe pose goal
    // 2. subscribe string name goal
    // 3. pub navigation status
    // 4. subscribe param,, set_vel, set_angular_vel
    // 5. Parse parameter, topic name, initial params

    if (GetPawn() && ROS2Interface && ROS2Interface->RobotROS2Node &&
        ROS2Interface->RobotROS2Node->State == UROS2State::Initialized)
    {
        ROS2_CREATE_SUBSCRIBER(ROS2Interface->RobotROS2Node,
                               this,
                               PoseGoalTopicName,
                               UROS2PoseStampedMsg::StaticClass(),
                               &ARRAIRobotROSController::PoseGoalCallback);
        ROS2_CREATE_SUBSCRIBER(ROS2Interface->RobotROS2Node,
                               this,
                               ActorGoalTopicName,
                               UROS2StrMsg::StaticClass(),
                               &ARRAIRobotROSController::ActorGoalCallback);
        ROS2_CREATE_LOOP_PUBLISHER(ROS2Interface->RobotROS2Node,
                                   this,
                                   NavStatusTopicName,
                                   UROS2Publisher::StaticClass(),
                                   UROS2Int32Msg::StaticClass(),
                                   NavStatusPublicationFrequencyHz,
                                   &ARRAIRobotROSController::UpdateNavStatus,
                                   NavStatusPublisher);
        ROS2_CREATE_SUBSCRIBER(ROS2Interface->RobotROS2Node,
                               this,
                               SetSpeedTopicName,
                               UROS2Float32Msg::StaticClass(),
                               &ARRAIRobotROSController::SetSpeedCallback);
        ROS2_CREATE_SUBSCRIBER(ROS2Interface->RobotROS2Node,
                               this,
                               SetAngularVelTopicName,
                               UROS2Float32Msg::StaticClass(),
                               &ARRAIRobotROSController::SetAngularVelCallback);
        ROS2_CREATE_SUBSCRIBER(ROS2Interface->RobotROS2Node,
                               this,
                               SetModeTopicName,
                               UROS2Int32Msg::StaticClass(),
                               &ARRAIRobotROSController::SetModeCallback);

        // Additional initialization implemented in child BP class.
        BPInitROS2InterfaceImpl();

        GetWorld()->GetTimerManager().ClearTimer(ROS2InitTimer);
        return true;
    }

    return false;
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
        FTransform worldTrans =
            URRGeneralUtils::GetWorldTransform(poseStamped.Header.FrameId,
                                               this,
                                               URRConversionUtils::TransformROSToUE(FTransform(
                                                   poseStamped.Pose.Orientation, poseStamped.Pose.Position, FVector::OneVector)));
        FMoveCompleteCallback onSuccess, onFail;    // dummy
        MoveToLocationWithDelegates(worldTrans.GetLocation(), worldTrans.GetRotation().Rotator(), onSuccess, onFail);
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

void ARRAIRobotROSController::SetSpeedCallback(const UROS2GenericMsg* Msg)
{
    const UROS2Float32Msg* floatMsg = Cast<UROS2Float32Msg>(Msg);
    if (IsValid(floatMsg))
    {
        FROSFloat32 floatValue;
        floatMsg->GetMsg(floatValue);
        float floatData = floatValue.Data;

        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("speed is changed to: %f"), floatData);
        SetSpeed(URRConversionUtils::DistanceROSToUE(floatData));
    }
}

void ARRAIRobotROSController::SetAngularVelCallback(const UROS2GenericMsg* Msg)
{
    const UROS2Float32Msg* floatMsg = Cast<UROS2Float32Msg>(Msg);
    if (IsValid(floatMsg))
    {
        FROSFloat32 floatValue;
        floatMsg->GetMsg(floatValue);
        float floatData = floatValue.Data;

        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("rotation rate is changed to: %f"), floatData);
        SetRotationRate(FMath::RadiansToDegrees(floatData));
    }
}

void ARRAIRobotROSController::SetModeCallback(const UROS2GenericMsg* Msg)
{
    const UROS2Int32Msg* intMsg = Cast<UROS2Int32Msg>(Msg);
    if (IsValid(intMsg))
    {
        FROSInt32 intValue;
        intMsg->GetMsg(intValue);
        int intData = intValue.Data;

        UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("mode value: %d"), intData);
        if (intData > (int)ERRAIRobotMode::BEGIN && intData < (int)ERRAIRobotMode::END)
        {
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Log, TEXT("mode value is changed to %d"), intData);
            Mode = static_cast<ERRAIRobotMode>(intData);
        }
        else
        {
            UE_LOG_WITH_INFO_NAMED(LogRapyutaCore, Warning, TEXT("invalid mode value %d are given."));
        }
    }
}

void ARRAIRobotROSController::ResetControl()
{
    NavStatus = ERRAIRobotNavStatus::IDLE;
    OnSuccess.Unbind();
    OnFail.Unbind();
}

bool ARRAIRobotROSController::InProgress()
{
    return NavStatus != ERRAIRobotNavStatus::IDLE;
}

void ARRAIRobotROSController::SetDelegates(const FMoveCompleteCallback& InOnSuccess,
                                           const FMoveCompleteCallback& InOnFail,
                                           const float InLinearMotionTolerance,
                                           const float InOrientationTolerance,
                                           const float InTimeOut,
                                           const bool Verbose)
{
    if (!InOnSuccess.IsBound())
    {
        if (Verbose)
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("OnSuccess Delegate is not set - is this on purpose? "));
        }
    }
    else
    {
        OnSuccess.Unbind();
        OnSuccess = InOnSuccess;
    }

    if (!InOnFail.IsBound())
    {
        if (Verbose)
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("OnFail Delegate is not set - is this on purpose? "));
        }
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
    if (NavStatus == ERRAIRobotNavStatus::AI_MOVING && PositionEquals(AIMovePositionTarget, Goal->GetActorLocation()))
    {
        UE_LOG_WITH_INFO_SHORT(LogTemp, Log, TEXT("Same Navigation position target is given. Ignore."));
        return EPathFollowingRequestResult::Type::Failed;
    }

    NavStatus = ERRAIRobotNavStatus::AI_MOVING;

    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    OrientationTarget = Goal->GetActorRotation();
    AIMovePositionTarget = Goal->GetActorLocation();    // for teleport on fail

    auto res =
        MoveToActor(Goal, LinearMotionTolerance, bStopOnOverlap, bUsePathfinding, bCanStrafe, FilterClass, bAllowPartialPath);
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
    FTransform worldDest = URRGeneralUtils::GetWorldTransform(FTransform(InOriginRotator, InOriginPosition, FVector::OneVector),
                                                              FTransform(DestRotator, Dest, FVector::OneVector));

    if (NavStatus == ERRAIRobotNavStatus::AI_MOVING && PositionEquals(AIMovePositionTarget, worldDest.GetLocation()))
    {
        UE_LOG_WITH_INFO_SHORT(LogTemp, Log, TEXT("Same Navigation position target is given. Ignore."));
        return EPathFollowingRequestResult::Type::Failed;
    }

    NavStatus = ERRAIRobotNavStatus::AI_MOVING;
    SetDelegates(InOnSuccess, InOnFail, AcceptanceRadius, InOrientationTolerance, InTimeOut);
    OrientationTarget = worldDest.GetRotation().Rotator();
    AIMovePositionTarget = worldDest.GetLocation();    // for teleport on fail

    auto res = MoveToLocation(Dest,
                              LinearMotionTolerance,
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
                GetPawn()->SetActorLocation(AIMovePositionTarget);
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
        AngularVelocity[i] = FMath::IsNearlyZero(angleDiff) ? 0 : angleDiff < 0 ? -RotationRate : RotationRate;
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

bool ARRAIRobotROSController::OrientationEquals(const FRotator InOrientationTarget1,
                                                const FRotator InOrientationTarget2,
                                                const float InOrientationTolerance)
{
    const float orientationTolerance = (InOrientationTolerance >= 0) ? InOrientationTolerance : OrientationTolerance;
    return InOrientationTarget2.Equals(InOrientationTarget1, orientationTolerance);
}

bool ARRAIRobotROSController::HasReachedOrientationTarget(const FRotator InOrientationTarget, const float InOrientationTolerance)
{
    return OrientationEquals(GetPawn()->GetActorRotation(), InOrientationTarget, InOrientationTolerance);
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

bool ARRAIRobotROSController::PositionEquals(const FVector InLinearMotionTarget1,
                                             const FVector InLinearMotionTarget2,
                                             const float InLinearMotionTolerance)
{
    const float linearMotionTolerance = (InLinearMotionTolerance >= 0) ? InLinearMotionTolerance : LinearMotionTolerance;
    return InLinearMotionTarget2.Equals(InLinearMotionTarget1, linearMotionTolerance);
}

bool ARRAIRobotROSController::HasReachedLinearMotionTarget(const FVector InLinearMotionTarget, const float InLinearMotionTolerance)
{
    return PositionEquals(GetPawn()->GetActorLocation(), InLinearMotionTarget, InLinearMotionTolerance);
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

FTransform ARRAIRobotROSController::GetOrigin()
{
    if (OriginActor)
    {
        return OriginActor->GetTransform();
    }
    else
    {
        return Origin;
    }
}

bool ARRAIRobotROSController::UpdateGoalSequenceIndex(FTransform& OutGoal)
{
    if (GoalSequence.Num() > 0)
    {
        GoalIndex++;
        if (GoalSequence.Num() <= GoalIndex)
        {
            GoalIndex = 0;
        }
        OutGoal = GoalSequence[GoalIndex];
        return true;
    }
    return false;
}

bool ARRAIRobotROSController::RandomUpdateGoalSequenceIndex(FTransform& OutGoal)
{
    if (GoalSequence.Num() > 0)
    {
        if (GoalSequence.Num() > 2)
        {
            int temp = FMath::RandRange(0, GoalSequence.Num() - 1);
            while (temp == GoalIndex)
            {
                temp = FMath::RandRange(0, GoalSequence.Num() - 1);
            }
            GoalIndex = temp;
            OutGoal = GoalSequence[GoalIndex];
            return true;
        }
    }
    return false;
}

void ARRAIRobotROSController::ModeUpdate()
{
    if (Mode != ERRAIRobotMode::MANUAL)
    {
        if (NavStatus == ERRAIRobotNavStatus::IDLE)
        {
            FMoveCompleteCallback onSuccess, onFail;    // dummy
            FTransform dummy;
            switch (Mode)
            {
                case ERRAIRobotMode::SEQUENCE:
                    if (GoalSequence.Num() > 0)
                    {
                        MoveToLocationWithDelegates(GoalSequence[GoalIndex].GetLocation(),
                                                    GoalSequence[GoalIndex].GetRotation().Rotator(),
                                                    onSuccess,
                                                    onFail);
                    }
                    UpdateGoalSequenceIndex(dummy);
                    break;
                case ERRAIRobotMode::RANDOM_SEQUENCE:
                    if (RandomUpdateGoalSequenceIndex(dummy))
                    {
                        MoveToLocationWithDelegates(GoalSequence[GoalIndex].GetLocation(),
                                                    GoalSequence[GoalIndex].GetRotation().Rotator(),
                                                    onSuccess,
                                                    onFail);
                    }
                    break;
                case ERRAIRobotMode::RANDOM_AREA:
                    MoveToLocationWithDelegates(
                        UKismetMathLibrary::RandomPointInBoundingBox(GetOrigin().GetLocation(), RandomMoveBoundingBox),
                        FRotator(0.0, UKismetMathLibrary::RandomRotator(true).Yaw, 0.0),
                        onSuccess,
                        onFail);
                    break;
                default:
                    Mode = ERRAIRobotMode::MANUAL;
                    break;
            }
        }
    }
}

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
        if (bModeUpdate)
        {
            ModeUpdate();
        }
    }
};
