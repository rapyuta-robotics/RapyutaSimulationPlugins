/**
 * @file JointComponent.h
 * @brief Base Joint component class which is used as part of #ARobotVehicle.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Components/ActorComponent.h"
#include "Core/RRStaticMeshComponent.h"
#include "CoreMinimal.h"

#include "RRJointComponent.generated.h"

#define RAPYUTA_JOINT_DEBUG (0)

DECLARE_DYNAMIC_DELEGATE(FJointCallback);

UENUM(BlueprintType)
enum class ERRJointControlType : uint8
{
    POSITION UMETA(DisplayName = "Position"),
    VELOCITY UMETA(DisplayName = "Velocity"),
    EFFORT UMETA(DisplayName = "Effort")
};

/**
 * @brief Base Joints class. Other joint class should inherit from this class.
 * 6 DOF joints class with velocity or position control.
 * temporary implementation of joints. should be merged with RobotImporter later.
 *
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRJointComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    URRJointComponent();

protected:
    virtual void BeginPlay() override;
    virtual void PoseFromArray(const TArray<float>& InPose, FVector& OutPosition, FRotator& OutOrientation);
    virtual void VelocityFromArray(const TArray<float>& InVelocity, FVector& OutLinearVelocity, FVector& OutAngularVelocity);

public:
    virtual bool IsValid();

    /**
     * @brief Call #UpdatePose after update #PositionTarget and #OrientationTarget with #LinearVelocity and AngularVelocity
     *
     * @param DeltaTime
     * @param TickType
     * @param ThisTickFunction
     */
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    /**
     * @brief Initialize #JointToChildLink and #ParentLinkToJoint
     * 
     */
    virtual void Initialize();

    /**
     * @brief Directly set velocity.
     * Control to move joint with this velocity should be implemented in child class.
     * @param InLinearVelocity
     * @param InAngularVelocity
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetVelocity(const FVector& InLinearVelocity, const FVector& InAngularVelocity);

    /**
     * @brief Set velocity target
     * Control to move joint with this velocity should be implemented in child class.
     * @param InLinearVelocity
     * @param InAngularVelocity
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetVelocityTarget(const FVector& InLinearVelocity, const FVector& InAngularVelocity);

    /**
     * @brief Utility wrapper of #SetVelocityTarget to provide 1 DoF target orientation
     * This is useful since most of the joint is 1 DOF
     *
     * @param Input
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSingleLinearVelocityTarget(const float Input);

    /**
     * @brief Utility wrapper of #SetVelocityTarget to provide 1 DoF target position
     * This is useful since most of the joint is 1 DOF
     *
     * @param Input
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSingleAngularVelocityTarget(const float Input);

    /**
     * @brief Set the Velocity Target With Delegates object
     *
     * @param InLinearVelocity
     * @param InAngularVelocity
     * @param InOnControlSuccessDelegate
     * @param InOnControlFailDelegate
     * @param InLinearTolerance
     * @param InAngularTolerance
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetVelocityTargetWithDelegates(const FVector& InLinearVelocity,
                                                const FVector& InAngularVelocity,
                                                const FJointCallback& InOnControlSuccessDelegate,
                                                const FJointCallback& InOnControlFailDelegate,
                                                const float InLinearTolerance = -1.0,
                                                const float InAngularTolerance = -1.0,
                                                const float InTimeOut = -1.0);

    /**
     * @brief Utility wrapper of #SetVelocityTargetWithDelegates to provide 1 DoF targetlinear  velocity
     * This is useful since most of the joint is 1 DOF
     *
     *
     * @param Input
     * @param InOnControlSuccessDelegate
     * @param InOnControlFailDelegate
     * @param InLinearTolerance
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSingleLinearVelocityTargetWithDelegates(const float Input,
                                                            const FJointCallback& InOnControlSuccessDelegate,
                                                            const FJointCallback& InOnControlFailDelegate,
                                                            const float InLinearTolerance = -1.0,
                                                            const float InTimeOut = -1.0);

    /**
     * @brief Utility wrapper of #SetVelocityTargetWithDelegates to provide 1 DoF target angular velocity
     * This is useful since most of the joint is 1 DOF
     *
     *
     * @param Input
     * @param InOnControlSuccessDelegate
     * @param InOnControlFailDelegate
     * @param InLinearTolerance
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSingleAngularVelocityTargetWithDelegates(const float Input,
                                                             const FJointCallback& InOnControlSuccessDelegate,
                                                             const FJointCallback& InOnControlFailDelegate,
                                                             const float InAngularTolerance = -1.0,
                                                             const float InTimeOut = -1.0);

    /**
     * @brief Set the Velocity With TArray.
     * Control to move joint with this velocity should be implemented in child class.
     * TArray size should be #LinearDOF +  #RotationalDOF
     * @param InVelocity
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetVelocityWithArray(const TArray<float>& InVelocity);

    /**
     * @brief Set the Velocity Target.
     * Control to move joint with this velocity should be implemented in child class.
     * TArray size should be #LinearDOF +  #RotationalDOF
     * @param InVelocity
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetVelocityTargetWithArray(const TArray<float>& InVelocity);

    /**
     * @brief Set the Velocity Target With Array With Delegates object
     *
     *
     * @param InVelocity
     * @param InOnControlSuccessDelegate
     * @param InOnControlFailDelegate
     * @param InLinearTolerance
     * @param InAngularTolerance
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetVelocityTargetWithArrayWithDelegates(const TArray<float>& InVelocity,
                                                         const FJointCallback& InOnControlSuccessDelegate,
                                                         const FJointCallback& InOnControlFailDelegate,
                                                         const float InLinearTolerance = -1.0,
                                                         const float InAngularTolerance = -1.0,
                                                         const float InTimeOut = -1.0);

    /**
     * @brief Check Pose reach the target pose.
     * If minus values are given or called without args, #LinearVelocityTolerance and #AngularVelocityTolerance will be used.
     */
    UFUNCTION(BlueprintCallable)
    virtual bool HasReachedVelocityTarget(const float InLinearTolerance = -1.f, const float InAngularTolerance = -1.f);

    /**
     * @brief Directly set pose.
     * Control to teleport joint to this pose should be implemented in child class.
     * @param InPosition
     * @param InOrientation
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetPose(const FVector& InPosition, const FRotator& InOrientation);

    /**
     * @brief Set Pose Target.
     * Control to move joint to pose target should be implemented in child class.
     * @param InPosition
     * @param InOrientation
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetPoseTarget(const FVector& InPosition, const FRotator& InOrientation);

    /**
     * @brief Utility wrapper of #SetPoseTarget to provide 1 DoF target orientation
     * This is useful since most of the joint is 1 DOF
     *
     * @param Input
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSinglePositionTarget(const float Input);

    /**
     * @brief Utility wrapper of #SetPoseTarget to provide 1 DoF target position
     * This is useful since most of the joint is 1 DOF
     *
     * @param Input
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSingleOrientationTarget(const float Input);

    /**
     * @brief Set Pose Target.
     *
     * @param InPosition
     * @param InOrientation
     * @param InOnControlSuccessDelegate called when move to target success
     * @param InOnControlFailDelegate called when move to target fail
     * @param InPositionTolerance  update #PositionTolerance if this is set >= 0.
     * @param InOrientationTolerance update #OrientationTolerance if this is set >= 0.
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetPoseTargetWithDelegates(const FVector& InPosition,
                                            const FRotator& InOrientation,
                                            const FJointCallback& InOnControlSuccessDelegate,
                                            const FJointCallback& InOnControlFailDelegate,
                                            const float InPositionTolerance = -1.0,
                                            const float InOrientationTolerance = -1.0,
                                            const float InTimeOut = -1.0);

    /**
     * @brief Utility wrapper of #SetPoseTargetWithDelegates to provide 1 DoF target position
     * This is useful since most of the joint is 1 DOF
     *
     * @param Input
     * @param InOnControlSuccessDelegate
     * @param InOnControlFailDelegate
     * @param InPositionTolerance
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSinglePositionTargetWithDelegates(const float Input,
                                                      const FJointCallback& InOnControlSuccessDelegate,
                                                      const FJointCallback& InOnControlFailDelegate,
                                                      const float InPositionTolerance = -1.0,
                                                      const float InTimeOut = -1.0);

    /**
     * @brief Utility wrapper of #SetPoseTargetWithDelegates to provide 1 DoF target orientation
     * This is useful since most of the joint is 1 DOF
     *
     * @param Input
     * @param InOnControlSuccessDelegate called when move to target success
     * @param InOnControlFailDelegate called when move to target fail
     * @param InOrientationTolerance update #OrientationTolerance if this is set >= 0.
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetSingleOrientationTargetWithDelegates(const float Input,
                                                         const FJointCallback& InOnControlSuccessDelegate,
                                                         const FJointCallback& InOnControlFailDelegate,
                                                         const float InOrientationTolerance = -1.0,
                                                         const float InTimeOut = -1.0);

    /**
     * @brief Check Pose reach the target pose.
     * If minus values are given, #PositionTolerance and #OrientationTolerance will be used.
     *
     */
    UFUNCTION(BlueprintCallable)
    virtual bool HasReachedPoseTarget(const float InPositionTolerance = -1.f, const float InOrientationTolerance = -1.f);

    /**
     * @brief Directly set pose.
     * Control to teleport joint to pose should be implemented in child class.
     * TArray size should be #LinearDOF +  #RotationalDOF
     * @param InPose
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetPoseWithArray(const TArray<float>& InPose);

    /**
     * @brief Set Pose Target.
     * Control to move joint to pose target should be implemented in child class.
     * TArray size should be #LinearDOF +  #RotationalDOF
     * @param InPose
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetPoseTargetWithArray(const TArray<float>& InPose);

    /**
     * @brief Set Pose Target.
     *
     * @param InPose
     * @param InOnControlSuccessDelegate
     * @param InOnControlFailDelegate
     * @param InTimeOut
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetPoseTargetWithArrayWithDelegates(const TArray<float>& InPose,
                                                     const FJointCallback& InOnControlSuccessDelegate,
                                                     const FJointCallback& InOnControlFailDelegate,
                                                     const float InPositionTolerance = -1.0,
                                                     const float InOrientationTolerance = -1.0,
                                                     const float InTimeOut = -1.0);

    /**
     * @brief Teleport robot to given pose. Implementation is in child class.
     * 
     */
    UFUNCTION(BlueprintCallable)
    virtual void Teleport(const FVector& InPosition, const FRotator& InOrientation);

    UFUNCTION(BlueprintCallable)
    virtual void MoveToInitPose();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelocity = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelocity = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelocityTarget = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelocityTarget = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PositionTarget = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationTarget = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Position = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator Orientation = FRotator::ZeroRotator;

    //! [cm] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float PositionTolerance = 1.f;

    //! [degree] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float OrientationTolerance = 1.f;

    //! [cm/s] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float LinearVelocityTolerance = 10.f;
    
    //! [degree/s] tolerance for control
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float AngularVelocityTolerance = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* ParentLink;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UStaticMeshComponent* ChildLink;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    ERRJointControlType ControlType = ERRJointControlType::POSITION;

    //! Linear Degrees Of Freedom
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 LinearDOF = 0;

    //! Rotational Degrees Of Freedom
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    uint8 RotationalDOF = 1;

    //! Pose Limitations[cm]
    //! @todo is it possible to set inf?
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PositionMax = FVector(1000, 1000, 1000);

    //! Pose Limitations[cm]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PositionMin = FVector(-1000, -1000, -1000);

    //! Orientation Limitations[deg]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationMax = FRotator(180.f);

    //! Orientation Limitations[deg]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator OrientationMin = FRotator(-180, -180, -180);

    //! Linear Velocity Limitations[cm/s]
    //! @todo is it possible to set inf?
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector LinearVelMax = FVector(1000, 1000, 1000);

    //! Angular Velocity Limitations[deg/s]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelMax = FVector(180.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bLimitRoll = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bLimitPitch = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bLimitYaw = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector InitialPosition = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator InitialOrientation = FRotator::ZeroRotator;

protected:
    UFUNCTION()
    virtual void UpdateState(const float DeltaTime);

    UFUNCTION()
    virtual void UpdateControl(const float DeltaTime);

    UPROPERTY()
    FTransform JointToChildLink = FTransform::Identity;

    UPROPERTY()
    FTransform ParentLinkToJoint = FTransform::Identity;

    //! true if joint is moving to target pose
    UPROPERTY(VisibleAnywhere)
    bool bMovingToTargetPose = false;

    //! true if joint is moving to target velocity
    UPROPERTY(VisibleAnywhere)
    bool bMovingToTargetVelocity = false;

    /**
    * @brief Set Delegates which will be unbounded when move is completed or timeouthappen
    *
    * @param InOnControlSuccessDelegate called when move to target success
    * @param InOnControlFailDelegate called when move to target fail
    * @param InPositionTolerance  update #PositionTolerance if this is set >= 0.
    * @param InOrientationTolerance update #OrientationTolerance if this is set >= 0.
    * @param InTimeOut if this is set less than 0, timeout won't happen.
    */
    virtual void SetDelegates(const FJointCallback& InOnControlSuccessDelegate,
                              const FJointCallback& InOnControlFailDelegate,
                              const float InPositionTolerance = -1.0,
                              const float InOrientationTolerance = -1.0,
                              const float InLinearTolerance = -1.0,
                              const float InAngularTolerance = -1.0,
                              const float InTimeOut = -1.0f);

    //! Delegate which is called when joint reach target vel/pose
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FJointCallback OnControlSuccessDelegate;

    //! Delegate which is called whenjoint failed to reach target vel/pose
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FJointCallback OnControlFailDelegate;

    //! time when target pose/vel are set.
    float ControlStartTime = 0.f;

    //! timeout. If joint can't reach target in this duration, OnControlFailDelegate should be called.
    //! used only with #SetPoseTargetWithDelegate
    //! if this is set less than 0, timeout won't happen.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ControlTimeout = -1.0f;
};
