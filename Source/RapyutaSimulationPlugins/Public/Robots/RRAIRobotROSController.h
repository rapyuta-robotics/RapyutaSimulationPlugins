/**
 * @file RRRobotVehicleROSController.h
 * @brief Base Robot ROS controller class. Other robot controller class should inherit from this class. Example is
 * #ATurtlebotROSController.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "AIController.h"
#include "CoreMinimal.h"
#include "Navigation/PathFollowingComponent.h"

// RapyutaSimulationPlugins
#include "Robots/RRBaseRobotROSController.h"

#include "RRAIRobotROSController.generated.h"

class URRRobotROS2Interface;

DECLARE_DYNAMIC_DELEGATE(FMoveCompleteCallback);
DECLARE_DELEGATE(FMoveCompleteCallbackStatic);

/**
 * @brief Robot navigation status
 *
 */
UENUM(BlueprintType)
enum class ERRAIRobotNavStatus : uint8
{
    IDLE UMETA(DisplayName = "Idle"),
    AI_MOVING UMETA(DisplayName = "AI moving"),
    LINEAR_MOVING UMETA(DisplayName = "Linear moving"),
    ROTATING UMETA(DisplayName = "Rotating")
};

/**
 * @brief AI agent mode status
 *
 */
UENUM(BlueprintType)
enum class ERRAIRobotMode : uint8
{
    BEGIN = 0 UMETA(DisplayName = "Begin"),

    MANUAL = 1 UMETA(DisplayName = "Manual"),
    SEQUENCE = 2 UMETA(DisplayName = "Sequence"),
    RANDOM_SEQUENCE = 3 UMETA(DisplayName = "Random Sequence"),
    RANDOM_AREA = 4 UMETA(DisplayName = "Random Area"),

    END = 100 UMETA(DisplayName = "End")
};

/**
 * @brief  Base Robot ROS controller class with AI movement by UE navigation system.
 * This class has authority to start ROS 2 Component in pausses robot.
 * This class implement basic motion unit such as AI movement and etc in C++.
 * Complicated motion should be implemented in BP by cobining behavior implemented in C++.
 *
 * @sa [AAIController](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/)
 * @sa https://answers.unrealengine.com/questions/871116/view.html
 * @sa https://answers.unrealengine.com/questions/239159/how-many-ai-controllers-should-i-have.html
 *
 * @todo not work in client-server
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRAIRobotROSController : public ARRBaseRobotROSController
{
    GENERATED_BODY()

public:
    //! debug flat
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bDebug = false;

    //! [degree] tolerance for orientation control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OrientationTolerance = 5.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearMotionTolerance = 10.f;

    //! Orientation target in ERRAIRobotMode::ROTATING
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationTarget = FRotator::ZeroRotator;

    //! Linear motion target in ERRAIRobotMode::LINEAR_MOVING
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearMotionTarget = FVector::ZeroVector;

    //! Teleport to target pose if robot can't reach target pose
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bTeleportOnFail = true;

    //! Set Speed to UFloatingPawnMovement or UCharacterMovementComponent
    //! @sa [UFloatingPawnMovement]<https://docs.unrealengine.com/5.2/en-US/API/Runtime/Engine/GameFramework/UFloatingPawnMovement/>
    //! @sa [UCharacterMovementComponent]<https://docs.unrealengine.com/5.3/en-US/API/Runtime/Engine/GameFramework/UCharacterMovementComponent/>
    UFUNCTION(BlueprintCallable)
    virtual bool SetSpeed(const float InSpeed);

    //! Set RotationRate toUCharacterMovementComponent
    //! @sa [UCharacterMovementComponent]<https://docs.unrealengine.com/5.3/en-US/API/Runtime/Engine/GameFramework/UCharacterMovementComponent/>
    UFUNCTION(BlueprintCallable)
    virtual bool SetRotationRate(const float InRotationRate);

    //! Set Acceleration to UFloatingPawnMovement
    UFUNCTION(BlueprintCallable)
    virtual bool SetAcceleration(const float InAcceleration);

    // ROS
    //! Publisher of ERRAIRobotNavStatus
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TObjectPtr<UROS2Publisher> NavStatusPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float NavStatusPublicationFrequencyHz = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString SetSpeedTopicName = TEXT("set_speed");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString SetAngularVelTopicName = TEXT("set_angular_vel");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString SetModeTopicName = TEXT("set_mode");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString NavStatusTopicName = TEXT("nav_status");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString PoseGoalTopicName = TEXT("pose_goal");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ActorGoalTopicName = TEXT("actor_goal");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    ERRAIRobotMode Mode = ERRAIRobotMode::MANUAL;

    //! Bounding box for random move. This is used when #Mode is ERRAIRobotMode::RANDOM_AREA
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector RandomMoveBoundingBox = FVector::OneVector;

    //! Goal sequence. This is used when #Mode is ERRAIRobotMode::SEQUENCE and ERRAIRobotMode::RANDOM_SEQUENCE
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FTransform> GoalSequence;

    //! Origin actor for move. This is used when #Mode is ERRAIRobotMode::RANDOM_AREA, ERRAIRobotMode::SEQUENCE and ERRAIRobotMode::RANDOM_SEQUENCE
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    AActor* OriginActor = nullptr;

    //! Origin transform for move. This is used when #Mode is ERRAIRobotMode::RANDOM_AREA, ERRAIRobotMode::SEQUENCE and ERRAIRobotMode::RANDOM_SEQUENCE
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform Origin = FTransform::Identity;

    //! Return origin transform from #OriginActor or #Origin
    UFUNCTION(BlueprintCallable)
    virtual FTransform GetOrigin();

    //! Init propertiese from JSON which is used when pawn is spawned from ROS2 spawn entity service.
    UFUNCTION(BlueprintCallable)
    bool InitPropertiesFromJSON();

protected:
    /**
     * @brief Initialize robot pawn by calling #ARRBaseRobot::InitROS2Interface.
     *
     * @sa [OnPossess](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/OnPossess/)
     * @param InPawn
     */
    virtual void OnPossess(APawn* InPawn) override;

    /**
     * @brief Deinitialize robot pawn by calling #ARRBaseRobot::DeInitROS2Interface.
     *
     * @sa [OnUnPossess](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/OnUnPossess/)
     */
    virtual void OnUnPossess() override;

    virtual void OnMoveCompleted(FAIRequestID RequestID, const FPathFollowingResult& Result) override;

    /**
     * @brief Target position for AI movement.
     *
     */
    UPROPERTY(VisibleAnywhere)
    FVector AIMovePositionTarget;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RotationRate = 90.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearSpeed = 100.f;

    UPROPERTY(VisibleAnywhere)
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    ERRAIRobotNavStatus NavStatus = ERRAIRobotNavStatus::IDLE;

    /**
    * @brief Set Delegates for move to target.
    *
    * @param InOnControlSuccessDelegate called when move to target success
    * @param InOnControlFailDelegate called when move to target fail
    * @param InPositionTolerance  update #PositionTolerance if this is set >= 0.
    * @param InOrientationTolerance update #OrientationTolerance if this is set >= 0.
    * @param InTimeOut if this is set less than 0, timeout won't happen.
    */
    virtual void SetDelegates(const FMoveCompleteCallback& InOnSuccess,
                              const FMoveCompleteCallback& InOnFail,
                              const float InLinearMotionToleranc = -1.0,
                              const float InOrientationTolerance = -1.0,
                              const float InTimeOut = -1.0f,
                              const bool Verbose = false);

    //! Delegate which is called when joint reach target vel/pose
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FMoveCompleteCallback OnSuccess;

    //! Delegate which is called on action completed internally
    FMoveCompleteCallbackStatic OnSuccessInternal;

    //! Delegate which is called whenjoint failed to reach target vel/pose
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FMoveCompleteCallback OnFail;

    /**
     * @brief Check controller is child class of #ARRAIRobotROSController or not
     *
     * @param TargetPawn
     * @return ARRAIRobotROSController*
     */
    static ARRAIRobotROSController* CheckController(APawn* TargetPawn);

    /**
     * @brief Rotate after AAIController::MoveToActor is completed and call InOnSuccess or InOnFail.
     * @sa [AAIController::MoveToActor](https://docs.unrealengine.com/5.2/en-US/API/Runtime/AIModule/AAIController/MoveToActor/)
     * @param Goal
     * @param InOnSuccess
     * @param InOnFail
     * @param AcceptanceRadius
     * @param bStopOnOverlap
     * @param bUsePathfinding
     * @param bCanStrafe
     * @param FilterClass
     * @param bAllowPartialPath
     * @param InOrientationTolerance
     * @param InTimeOut
     * @return EPathFollowingRequestResult::Type
     */
    virtual EPathFollowingRequestResult::Type MoveToActorWithDelegates(AActor* Goal,
                                                                       const FMoveCompleteCallback& InOnSuccess,
                                                                       const FMoveCompleteCallback& InOnFail,
                                                                       float AcceptanceRadius = -1,
                                                                       bool bStopOnOverlap = true,
                                                                       bool bUsePathfinding = true,
                                                                       bool bCanStrafe = true,
                                                                       TSubclassOf<UNavigationQueryFilter> FilterClass = NULL,
                                                                       bool bAllowPartialPath = true,
                                                                       const float InOrientationTolerance = -1.0,
                                                                       const float InTimeOut = -1.0);

    /**
     * @brief Rotate after AAIController::MoveToActor is completed and call InOnSuccess or InOnFail.
     * @sa [AAIController::MoveToActor](https://docs.unrealengine.com/5.2/en-US/API/Runtime/AIModule/AAIController/MoveToActor/)
     * @param TargetPawn
     * @param Goal
     * @param InOnSuccess
     * @param InOnFail
     * @param AcceptanceRadius
     * @param bStopOnOverlap
     * @param bUsePathfinding
     * @param bCanStrafe
     * @param FilterClass
     * @param bAllowPartialPath
     * @param InOrientationTolerance
     * @param InTimeOut
     * @return EPathFollowingRequestResult::Type
     */
    UFUNCTION(BlueprintCallable,
              Category = "AI|Navigation",
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static EPathFollowingRequestResult::Type MoveToActorWithDelegates(APawn* TargetPawn,
                                                                      AActor* Goal,
                                                                      const FMoveCompleteCallback& InOnSuccess,
                                                                      const FMoveCompleteCallback& InOnFail,
                                                                      float AcceptanceRadius = -1,
                                                                      bool bStopOnOverlap = true,
                                                                      bool bUsePathfinding = true,
                                                                      bool bCanStrafe = true,
                                                                      TSubclassOf<UNavigationQueryFilter> FilterClass = NULL,
                                                                      bool bAllowPartialPath = true,
                                                                      const float InOrientationTolerance = -1.0,
                                                                      const float InTimeOut = -1.0);

    /**
     * @brief Rotate after AAIController::MoveToLocation is completed and call InOnSuccess or InOnFail.
     * @sa [AAIController::MoveToLocation](https://docs.unrealengine.com/5.2/en-US/API/Runtime/AIModule/AAIController/MoveToLocation/)
     * @param Dest
     * @param DestRotator
     * @param InOnSuccess
     * @param InOnFail
     * @param AcceptanceRadius
     * @param bStopOnOverlap
     * @param bUsePathfinding
     * @param bProjectDestinationToNavigation
     * @param bCanStrafe
     * @param FilterClass
     * @param bAllowPartialPath
     * @param InOrientationTolerance
     * @param InTimeOut
     * @param InOriginPosition
     * @param InOriginRotator
     * @return EPathFollowingRequestResult::Type
     */
    virtual EPathFollowingRequestResult::Type MoveToLocationWithDelegates(const FVector& Dest,
                                                                          const FRotator& DestRotator,
                                                                          const FMoveCompleteCallback& InOnSuccess,
                                                                          const FMoveCompleteCallback& InOnFail,
                                                                          float AcceptanceRadius = -1,
                                                                          bool bStopOnOverlap = true,
                                                                          bool bUsePathfinding = true,
                                                                          bool bProjectDestinationToNavigation = false,
                                                                          bool bCanStrafe = true,
                                                                          TSubclassOf<UNavigationQueryFilter> FilterClass = NULL,
                                                                          bool bAllowPartialPath = true,
                                                                          const float InOrientationTolerance = -1.0,
                                                                          const float InTimeOut = -1.0,
                                                                          const FVector& InOriginPosition = FVector::ZeroVector,
                                                                          const FRotator& InOriginRotator = FRotator::ZeroRotator);

    UFUNCTION(BlueprintCallable,
              Category = "AI|Navigation",
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    /**
     * @brief Rotate after AAIController::MoveToLocation is completed and call InOnSuccess or InOnFail.
     * @sa [AAIController::MoveToLocation](https://docs.unrealengine.com/5.2/en-US/API/Runtime/AIModule/AAIController/MoveToLocation/)
     * @param TargetPawn
     * @param Dest
     * @param DestRotator
     * @param InOnSuccess
     * @param InOnFail
     * @param AcceptanceRadius
     * @param bStopOnOverlap
     * @param bUsePathfinding
     * @param bProjectDestinationToNavigation
     * @param bCanStrafe
     * @param FilterClass
     * @param bAllowPartialPath
     * @param InOrientationTolerance
     * @param InTimeOut
     * @param InOriginPosition
     * @param InOriginRotator
     * @return EPathFollowingRequestResult::Type
     */
    static EPathFollowingRequestResult::Type MoveToLocationWithDelegates(APawn* TargetPawn,
                                                                         const FVector& Dest,
                                                                         const FRotator& DestRotator,
                                                                         const FMoveCompleteCallback& InOnSuccess,
                                                                         const FMoveCompleteCallback& InOnFail,
                                                                         float AcceptanceRadius = -1,
                                                                         bool bStopOnOverlap = true,
                                                                         bool bUsePathfinding = true,
                                                                         bool bProjectDestinationToNavigation = false,
                                                                         bool bCanStrafe = true,
                                                                         TSubclassOf<UNavigationQueryFilter> FilterClass = NULL,
                                                                         bool bAllowPartialPath = true,
                                                                         const float InOrientationTolerance = -1.0,
                                                                         const float InTimeOut = -1.0,
                                                                         const FVector& InOriginPosition = FVector::ZeroVector,
                                                                         const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief 1)Rotate, 2) Move forward to Dest, 3) Rotate to DestRotator and call InOnSuccess or InOnFail. This does not use AIMove in UE but just rotate and linear movement.
     *
     * @param Dest
     * @param DestRotator
     * @param InOnSuccess
     * @param InOnFail
     * @param AcceptanceRadius
     * @param InOrientationTolerance
     * @param InTimeOut
     * @param InOriginPosition
     * @param InOriginRotator
     * @return true
     * @return false
     */
    virtual bool LinearMoveToLocationWithDelegates(const FVector& Dest,
                                                   const FRotator& DestRotator,
                                                   const FMoveCompleteCallback& InOnSuccess,
                                                   const FMoveCompleteCallback& InOnFail,
                                                   float AcceptanceRadius = -1,
                                                   const float InOrientationTolerance = -1.0,
                                                   const float InTimeOut = -1.0,
                                                   const FVector& InOriginPosition = FVector::ZeroVector,
                                                   const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief 1)Rotate, 2) Move forward to Dest, 3) Rotate to DestRotator and call InOnSuccess or InOnFail. This does not use AIMove in UE but just rotate and linear movement.
     *
     * @param TargetPawn
     * @param Dest
     * @param DestRotator
     * @param InOnSuccess
     * @param InOnFail
     * @param AcceptanceRadius
     * @param InOrientationTolerance
     * @param InTimeOut
     * @param InOriginPosition
     * @param InOriginRotator
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool LinearMoveToLocationWithDelegates(APawn* TargetPawn,
                                                  const FVector& Dest,
                                                  const FRotator& DestRotator,
                                                  const FMoveCompleteCallback& InOnSuccess,
                                                  const FMoveCompleteCallback& InOnFail,
                                                  float AcceptanceRadius = -1,
                                                  const float InOrientationTolerance = -1.0,
                                                  const float InTimeOut = -1.0,
                                                  const FVector& InOriginPosition = FVector::ZeroVector,
                                                  const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief  Check orientation reach the target orientation and call InOnSuccess or InOnFail if it reach target.
     *
     * @param InOrientationTolerance If minus values are given, #OrientationTolerance will be used.
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable)
    virtual bool HasReachedOrientationTarget(const float InOrientationTolerance = -1.0);

    /**
     * @brief Check orientation reach the target orientation.
     *
     * @param InOrientationTarget
     * @param InOrientationTolerance If minus values are given, #OrientationTolerance will be used.
     * @return true
     * @return false
     */
    virtual bool HasReachedOrientationTarget(const FRotator InOrientationTarget, const float InOrientationTolerance = -1.0);

    /**
     * @brief Check InOrientationTarget1 and InOrientationTarget2 are equal.
     *
     * @param InOrientationTarget1
     * @param InOrientationTarget2
     * @param InOrientationTolerance If minus values are given, #OrientationTolerance will be used.
     * @return true
     * @return false
     */
    virtual bool OrientationEquals(const FRotator InOrientationTarget1,
                                   const FRotator InOrientationTarget2,
                                   const float InOrientationTolerance = -1.0);

    /**
     * @brief Check location reach the target location and call InOnSuccess or InOnFail if it reach target.
     *
     * @param InLinearMotionTolerance  If minus values are given, #LinearMotionTolerance will be used.
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable)
    virtual bool HasReachedLinearMotionTarget(const float InLinearMotionTolerance = -1.0);

    /**
     * @brief Check location reach the target location.
     *
     * @param InLinearMotionTarget
     * @param InLinearMotionTolerance  If minus values are given, #LinearMotionTolerance will be used.
     * @return true
     * @return false
     */
    virtual bool HasReachedLinearMotionTarget(const FVector InLinearMotionTarget, const float InLinearMotionTolerance = -1.0);

    /**
     * @brief Check InLinearMotionTarget1 and InLinearMotionTarget2 are equal.
     *
     * @param InLinearMotionTarget1
     * @param InLinearMotionTarget2
     * @param InLinearMotionTolerance  If minus values are given, #LinearMotionTolerance will be used.
     * @return true
     * @return false
     */
    virtual bool PositionEquals(const FVector InLinearMotionTarget1,
                                const FVector InLinearMotionTarget2,
                                const float InLinearMotionTolerance = -1.0);

    /**
     * @brief Set the Orientation Target and start rotating.
     *
     * @param InOrientation
     * @param InReset
     * @param InOriginRotator
     * @return true
     * @return false
     */
    virtual bool SetOrientationTarget(const FRotator& InOrientation,
                                      const bool InReset = true,
                                      const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Orientation Target from current location and start rotating.
     *
     * @param InOrientation
     * @param InReset
     * @return true
     * @return false
     */
    virtual bool AddLocalOrientationOffset(const FRotator& InOrientation, const bool InReset = true);

    /**
    * @brief Set the Orientation Target and start rotating
    *
    * @param TargetPawn
    * @param InOrientation
    * @param InReset
    * @param InOriginRotator
    * @return true
    * @return false
    */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool SetOrientationTarget(APawn* TargetPawn,
                                     const FRotator& InOrientation,
                                     const bool InReset = true,
                                     const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Orientation Target from current location and start rotating.
     *
     * @param TargetPawn
     * @param InOrientation
     * @param InReset
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool AddLocalOrientationOffset(APawn* TargetPawn, const FRotator& InOrientation, const bool InReset = true);

    /**
     * @brief  Set the Orientation Target and start rotating. InOnSuccess or InOnFail is called when it completed.
     *
     * @param InOrientation
     * @param InOnSuccess
     * @param InOnFail
     * @param InOrientationTolerance
     * @param InTimeOut
     * @param InOriginRotator
     * @return true
     * @return false
     */
    virtual bool SetOrientationTargetWthDelegates(const FRotator& InOrientation,
                                                  const FMoveCompleteCallback& InOnSuccess,
                                                  const FMoveCompleteCallback& InOnFail,
                                                  const float InOrientationTolerance = -1.0,
                                                  const float InTimeOut = -1.0,
                                                  const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Orientation Target from current location and start rotating. InOnSuccess or InOnFail is called when it completed.
     *
     * @param InOrientation
     * @param InOnSuccess
     * @param InOnFail
     * @param InOrientationTolerance
     * @param InTimeOut
     * @return true
     * @return false
     */
    virtual bool AddLocalOrientationOffsetWthDelegates(const FRotator& InOrientation,
                                                       const FMoveCompleteCallback& InOnSuccess,
                                                       const FMoveCompleteCallback& InOnFail,
                                                       const float InOrientationTolerance = -1.0,
                                                       const float InTimeOut = -1.0);

    /**
     * @brief Set the Orientation Target and start rotating. InOnSuccess or InOnFail is called when it completed.
     *
     * @param TargetPawn
     * @param InOrientation
     * @param InOnSuccess
     * @param InOnFail
     * @param InOrientationTolerance
     * @param InTimeOut
     * @param InOriginRotator
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool SetOrientationTargetWthDelegates(APawn* TargetPawn,
                                                 const FRotator& InOrientation,
                                                 const FMoveCompleteCallback& InOnSuccess,
                                                 const FMoveCompleteCallback& InOnFail,
                                                 const float InOrientationTolerance = -1.0,
                                                 const float InTimeOut = -1.0,
                                                 const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Orientation Target from current location and start rotating. InOnSuccess or InOnFail is called when it completed.
     *
     * @param TargetPawn
     * @param InOrientation
     * @param InOnSuccess
     * @param InOnFail
     * @param InOrientationTolerance
     * @param InTimeOut
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool AddLocalOrientationOffsetWthDelegates(APawn* TargetPawn,
                                                      const FRotator& InOrientation,
                                                      const FMoveCompleteCallback& InOnSuccess,
                                                      const FMoveCompleteCallback& InOnFail,
                                                      const float InOrientationTolerance = -1.0,
                                                      const float InTimeOut = -1.0);

    /**
     * @brief Set the Linear Motion Target and start moving.
     *
     * @param InPosition
     * @param InReset
     * @param InOriginPosition
     * @param InOriginRotator
     * @return true
     * @return false
     */
    virtual bool SetLinearMotionTarget(const FVector& InPosition,
                                       const bool InReset = true,
                                       const FVector& InOriginPosition = FVector::ZeroVector,
                                       const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Linear Motion Target from current location and start moving.
     *
     * @param InPosition
     * @param InReset
     * @return true
     * @return false
     */
    virtual bool AddLocalLinearMotionOffset(const FVector& InPosition, const bool InReset = true);

    /**
     * @brief Set the Linear Motion Target and start moving.
     *
     * @param TargetPawn
     * @param InPosition
     * @param InReset
     * @param InOriginPosition
     * @param InOriginRotator
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool SetLinearMotionTarget(APawn* TargetPawn,
                                      const FVector& InPosition,
                                      const bool InReset = true,
                                      const FVector& InOriginPosition = FVector::ZeroVector,
                                      const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Linear Motion Target from current location and start moving.
     *
     * @param TargetPawn
     * @param InPosition
     * @param InReset
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool AddLocalLinearMotionOffset(APawn* TargetPawn, const FVector& InPosition, const bool InReset = true);

    /**
     * @brief Set the Linear Motion Target and start moving. InOnSuccess or InOnFail is called when it completed.
     *
     * @param InPosition
     * @param InOnSuccess
     * @param InOnFail
     * @param InLinearMotionTolerancee
     * @param InTimeOut
     * @param InOriginPosition
     * @param InOriginRotator
     * @return true
     * @return false
     */
    virtual bool SetLinearMotionTargetWthDelegates(const FVector& InPosition,
                                                   const FMoveCompleteCallback& InOnSuccess,
                                                   const FMoveCompleteCallback& InOnFail,
                                                   const float InLinearMotionTolerancee = -1.0,
                                                   const float InTimeOut = -1.0,
                                                   const FVector& InOriginPosition = FVector::ZeroVector,
                                                   const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Linear Motion Target from current location and start moving. InOnSuccess or InOnFail is called when it completed.
     *
     * @param InPosition
     * @param InOnSuccess
     * @param InOnFail
     * @param InLinearMotionTolerancee
     * @param InTimeOut
     * @return true
     * @return false
     */
    virtual bool AddLocalLinearMotionOffsetWthDelegates(const FVector& InPosition,
                                                        const FMoveCompleteCallback& InOnSuccess,
                                                        const FMoveCompleteCallback& InOnFail,
                                                        const float InLinearMotionTolerancee = -1.0,
                                                        const float InTimeOut = -1.0);

    /**
     * @brief Set the Linear Motion Target and start moving. InOnSuccess or InOnFail is called when it completed.
     *
     * @param TargetPawn
     * @param InPosition
     * @param InOnSuccess
     * @param InOnFail
     * @param InLinearMotionTolerancee
     * @param InTimeOut
     * @param InOriginPosition
     * @param InOriginRotator
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool SetLinearMotionTargetWthDelegates(APawn* TargetPawn,
                                                  const FVector& InPosition,
                                                  const FMoveCompleteCallback& InOnSuccess,
                                                  const FMoveCompleteCallback& InOnFail,
                                                  const float InLinearMotionTolerancee = -1.0,
                                                  const float InTimeOut = -1.0,
                                                  const FVector& InOriginPosition = FVector::ZeroVector,
                                                  const FRotator& InOriginRotator = FRotator::ZeroRotator);

    /**
     * @brief Set the relative Linear Motion Target from current location and start moving. InOnSuccess or InOnFail is called when it completed.
     *
     * @param TargetPawn
     * @param InPosition
     * @param InOnSuccess
     * @param InOnFail
     * @param InLinearMotionTolerancee
     * @param InTimeOut
     * @return true
     * @return false
     */
    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static bool AddLocalLinearMotionOffsetWthDelegates(APawn* TargetPawn,
                                                       const FVector& InPosition,
                                                       const FMoveCompleteCallback& InOnSuccess,
                                                       const FMoveCompleteCallback& InOnFail,
                                                       const float InLinearMotionTolerancee = -1.0,
                                                       const float InTimeOut = -1.0);

    //! time when target pose/vel are set.
    float MoveStartTime = 0.f;

    //! timeout. If Pawn can't reach target in this duration, OnFail should be called.
    //! used only with #SetPoseTargetWithDelegate
    //! if this is set less than 0, timeout won't happen.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MoveTimeout = -1.0f;

    virtual void Tick(float DeltaSeconds) override;

    /**
     * @brief #UpdateRotation and #UpdateLinearMotion
     *
     * @param DeltaSeconds
     */
    virtual void UpdateControl(float DeltaSeconds);

    /**
     * @brief  Update oorientation motion when #NavStatus is ERRAIRobotNavStatus::ROTATING
     *
     * @param DeltaSeconds
     */
    virtual void UpdateRotation(float DeltaSeconds);

    /**
     * @brief Update linear motion when #NavStatus is ERRAIRobotNavStatus::LINEAR_MOVING
     *
     * @param DeltaSeconds
     */
    virtual void UpdateLinearMotion(float DeltaSeconds);

    /**
     * @brief Unbind delegates and reset status.
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual void ResetControl();

    /**
     * @brief is #NavStatus IDLE or not
     *
     * @return virtial
     */
    UFUNCTION(BlueprintCallable)
    virtual bool InProgress();

    // ROS
    //! Timer to call #InitROS2Interface
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle ROS2InitTimer;

    /**
     * @brief Initialize Parameter and start timer to call #InitROS2InterfaceImpl
     *
     */
    UFUNCTION()
    void InitROS2Interface();

    /**
     * @brief Create publishers and subscribers
     *
     */
    UFUNCTION()
    bool InitROS2InterfaceImpl();

    /**
     * @brief AdditionalInitialization implemented in BP.
     * Child BP class can use this method to add initialization behaviour
     */
    UFUNCTION(BlueprintImplementableEvent, BlueprintCallable)
    void BPInitROS2InterfaceImpl();

    /**
     * @brief Update NavStatus msg before publishing.
     *
     * @param InMessage
     */
    UFUNCTION()
    void UpdateNavStatus(UROS2GenericMsg* InMessage);

    /**
     * @brief Call MoveToLocationWithDelegates with given msg
     *
     * @param Msg
     */
    UFUNCTION()
    void PoseGoalCallback(const UROS2GenericMsg* Msg);

    /**
     * @brief Call MoveToActorWithDelegates with given msg
     *
     * @param Msg
     */
    UFUNCTION()
    void ActorGoalCallback(const UROS2GenericMsg* Msg);

    /**
     * @brief Call SetSpeed with given msg
     *
     * @param Msg
     */
    UFUNCTION()
    void SetSpeedCallback(const UROS2GenericMsg* Msg);

    /**
     * @brief Call SetRotationRate with given msg
     *
     * @param Msg
     */
    UFUNCTION()
    void SetAngularVelCallback(const UROS2GenericMsg* Msg);

    /**
     * @brief Call SetMode with given msg
     *
     * @param Msg
     */
    void SetModeCallback(const UROS2GenericMsg* Msg);

    // Mode variable
    //! Current goal index in #GoalSequence used in ERRAIRobotMode::SEQUENCE and ERRAIRobotMode::RANDOM_SEQUENCE
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int GoalIndex = 0;

    //! Mode transition in Tick
    //! @deprecated Implemented in Behavior Tree in Child BP class
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bModeUpdate = true;

    /**
     * @brief Increment #GoalIndex
     *
     * @return true
     * @return false if #GoalSequence is empty.
     */
    UFUNCTION(BlueprintCallable)
    bool UpdateGoalSequenceIndex(FTransform& OutGoal);

    /**
     * @brief Update #GoalIndex randomly
     *
     * @return true if #GoalSequence is not empty and #GoalIndex is > 2
     * @return false
     */
    UFUNCTION(BlueprintCallable)
    bool RandomUpdateGoalSequenceIndex(FTransform& OutGoal);

    /**
     * @brief
     * Mode transition function. This is called in #Tick.
     * @deprecated Implemented in Behavior Tree in Child BP class
     */
    UFUNCTION()
    virtual void ModeUpdate();
};
