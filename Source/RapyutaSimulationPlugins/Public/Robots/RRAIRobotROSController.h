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
#include "Robots/RRBaseRobotROSController.h"

#include "RRAIRobotROSController.generated.h"

class URRRobotROS2Interface;

DECLARE_DYNAMIC_DELEGATE(FMoveCompleteCallback);
DECLARE_DELEGATE(FMoveCompleteCallbackStatic);

/**
 * @brief  Base Robot ROS controller class. Other robot controller class should inherit from this class.
 * This class has authority to start ROS 2 Component in pausses robot.
 *
 * @sa [AAIController](https://docs.unrealengine.com/5.1/en-US/API/Runtime/AIModule/AAIController/)
 * @sa https://answers.unrealengine.com/questions/871116/view.html
 * @sa https://answers.unrealengine.com/questions/239159/how-many-ai-controllers-should-i-have.html
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ARRAIRobotROSController : public ARRBaseRobotROSController
{
    GENERATED_BODY()

public:
    //! [degree] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OrientationTolerance = 5.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearMotionTolerance = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationTarget = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearMotionTarget = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bTeleportOnFail = true;

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

    UPROPERTY(VisibleAnywhere)
    FVector AIMovePoseTarget;

    //! true if actor is moving to target pose
    UPROPERTY(VisibleAnywhere)
    bool bLinearMoving = false;

    //! true if actor is rotating to target orientation
    UPROPERTY(VisibleAnywhere)
    bool bRotating = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RotationSpeed = 90.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearSpeed = 100.f;

    UPROPERTY(VisibleAnywhere)
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere)
    FVector AngularVelocity = FVector::ZeroVector;

    /**
    * @brief Set Delegates which will be unbounded when move is completed or timeouthappen
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
                              const float InTimeOut = -1.0f);

    //! Delegate which is called when joint reach target vel/pose
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FMoveCompleteCallback OnSuccess;

    //! Delegate which is called on action completed internally
    FMoveCompleteCallbackStatic OnSuccessInternal;

    //! Delegate which is called whenjoint failed to reach target vel/pose
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FMoveCompleteCallback OnFail;

    static ARRAIRobotROSController* CheckController(APawn* TargetPawn);

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
                                                                          const FVector& OriginPosition = FVector::ZeroVector,
                                                                          const FRotator& OriginRotator = FRotator::ZeroRotator);

    UFUNCTION(BlueprintCallable,
              Category = "AI|Navigation",
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
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
                                                                         const FVector& OriginPosition = FVector::ZeroVector,
                                                                         const FRotator& OriginRotator = FRotator::ZeroRotator);

    virtual void LinearMoveToLocationWithDelegates(const FVector& Dest,
                                                   const FRotator& DestRotator,
                                                   const FMoveCompleteCallback& InOnSuccess,
                                                   const FMoveCompleteCallback& InOnFail,
                                                   float AcceptanceRadius = -1,
                                                   const float InOrientationTolerance = -1.0,
                                                   const float InTimeOut = -1.0);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void LinearMoveToLocationWithDelegates(APawn* TargetPawn,
                                                  const FVector& Dest,
                                                  const FRotator& DestRotator,
                                                  const FMoveCompleteCallback& InOnSuccess,
                                                  const FMoveCompleteCallback& InOnFail,
                                                  float AcceptanceRadius = -1,
                                                  const float InOrientationTolerance = -1.0,
                                                  const float InTimeOut = -1.0);
    /**
     * @brief Check orientation reach the target orientation.
     * If minus values are given, #OrientationTolerance will be used.
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual bool HasReachedOrientationTarget(const float InOrientationTolerance = -1.0);

    UFUNCTION(BlueprintCallable)
    virtual bool HasReachedLinearMotionTarget(const float InLinearMotionTolerance = -1.0);

    virtual void SetOrientationTarget(const FRotator& InOrientation, const bool InReset = true);

    virtual void AddLocalOrientationOffset(const FRotator& InOrientation, const bool InReset = true);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void SetOrientationTarget(APawn* TargetPawn, const FRotator& InOrientation);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void AddLocalOrientationOffset(APawn* TargetPawn, const FRotator& InOrientation);

    virtual void SetOrientationTargetWthDelegates(const FRotator& InOrientation,
                                                  const FMoveCompleteCallback& InOnSuccess,
                                                  const FMoveCompleteCallback& InOnFail,
                                                  const float InOrientationTolerance = -1.0,
                                                  const float InTimeOut = -1.0);

    virtual void AddLocalOrientationOffsetWthDelegates(const FRotator& InOrientation,
                                                       const FMoveCompleteCallback& InOnSuccess,
                                                       const FMoveCompleteCallback& InOnFail,
                                                       const float InOrientationTolerance = -1.0,
                                                       const float InTimeOut = -1.0);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void SetOrientationTargetWthDelegates(APawn* TargetPawn,
                                                 const FRotator& InOrientation,
                                                 const FMoveCompleteCallback& InOnSuccess,
                                                 const FMoveCompleteCallback& InOnFail,
                                                 const float InOrientationTolerance = -1.0,
                                                 const float InTimeOut = -1.0);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void AddLocalOrientationOffsetWthDelegates(APawn* TargetPawn,
                                                      const FRotator& InOrientation,
                                                      const FMoveCompleteCallback& InOnSuccess,
                                                      const FMoveCompleteCallback& InOnFail,
                                                      const float InOrientationTolerance = -1.0,
                                                      const float InTimeOut = -1.0);

    virtual void SetLinearMotionTarget(const FVector& InPosition, const bool InReset = true);

    virtual void AddLocalLinearMotionOffset(const FVector& InPosition, const bool InReset = true);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void SetLinearMotionTarget(APawn* TargetPawn, const FVector& InPosition);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void AddLocalLinearMotionOffset(APawn* TargetPawn, const FVector& InPosition);

    virtual void SetLinearMotionTargetWthDelegates(const FVector& InPosition,
                                                   const FMoveCompleteCallback& InOnSuccess,
                                                   const FMoveCompleteCallback& InOnFail,
                                                   const float InLinearMotionTolerancee = -1.0,
                                                   const float InTimeOut = -1.0);

    virtual void AddLocalLinearMotionOffsetWthDelegates(const FVector& InPosition,
                                                        const FMoveCompleteCallback& InOnSuccess,
                                                        const FMoveCompleteCallback& InOnFail,
                                                        const float InLinearMotionTolerancee = -1.0,
                                                        const float InTimeOut = -1.0);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void SetLinearMotionTargetWthDelegates(APawn* TargetPawn,
                                                  const FVector& InPosition,
                                                  const FMoveCompleteCallback& InOnSuccess,
                                                  const FMoveCompleteCallback& InOnFail,
                                                  const float InLinearMotionTolerancee = -1.0,
                                                  const float InTimeOut = -1.0);

    UFUNCTION(BlueprintCallable,
              meta = (DefaultToSelf = "TargetPawn", AdvancedDisplay = "bStopOnOverlap,bCanStrafe,bAllowPartialPath"))
    static void AddLocalLinearMotionOffsetWthDelegates(APawn* TargetPawn,
                                                       const FVector& InPosition,
                                                       const FMoveCompleteCallback& InOnSuccess,
                                                       const FMoveCompleteCallback& InOnFail,
                                                       const float InLinearMotionTolerancee = -1.0,
                                                       const float InTimeOut = -1.0);

    //! time when target pose/vel are set.
    float MoveStartTime = 0.f;

    //! timeout. If joint can't reach target in this duration, OnControlFailDelegate should be called.
    //! used only with #SetPoseTargetWithDelegate
    //! if this is set less than 0, timeout won't happen.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MoveTimeout = -1.0f;

    virtual void Tick(float DeltaSeconds) override;

    virtual void UpdateControl(float DeltaSeconds);

    virtual void UpdateRotation(float DeltaSeconds);

    virtual void UpdateLinearMotion(float DeltaSeconds);

    UFUNCTION(BlueprintCallable)
    virtual void ResetControl();
};
