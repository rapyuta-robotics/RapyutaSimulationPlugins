/**
 * @file RRROS2TFPublisher.h
 * @brief TF publisher class.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "Kismet/GameplayStatics.h"
#include "Math/TransformNonVectorized.h"

// rclUE
#include "Msgs/ROS2GenericMsg.h"
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Publisher.h"

// RapyutaSimulationPlugins
#include "Core/RRGeneralUtils.h"

#include "RRROS2TFPublisher.generated.h"

/**
 * @brief TF Publisher base class.
 * @sa [UROS2Publisher](https://rclue.readthedocs.io/en/devel/doxygen_generated/html/d6/dd4/class_u_r_o_s2_publisher.html)
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFPublisherBase : public UROS2Publisher
{
    GENERATED_BODY()

public:
    /**
    * @brief Construct a new URRROS2TFPublisher object
    *
    */
    URRROS2TFPublisherBase();

    //! Publish static tf or not.
    //! @sa https://docs.ros.org/en/rolling/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html?highlight=static%20tf
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool IsStatic = false;

    /**
     * @brief Initialize publisher with QoS
     *
     * @param InROS2Node
     */
    bool InitializeWithROS2(UROS2NodeComponent* InROS2Node) override;

    /**
     * @brief Update message frorm #TF.
     *
     * @param InMessage
     */
    virtual void UpdateMessage(UROS2GenericMsg* InMessage) override;

protected:
    /**
     * @brief Add tf data to OutROSTf from InFrameId, InChildFrameId, and InTf
     *
     * @param OutROSTf
     * @param InFrameId
     * @param InChildFrameId
     * @param InTf
     */
    virtual void AddTFtoMsg(FROSTFMsg& OutROSTf, const FString& InFrameId, const FString& InChildFrameId, const FTransform& InTf);

    /**
     * @brief Add tf data to OutROSTf from URRROS2TFComponent
     *
     * @param OutROSTf
     * @param InTfc
     */
    virtual void AddTFtoMsg(FROSTFMsg& OutROSTf, URRROS2TFComponent* InTfc);

    /**
     * @brief Add necessary tf data to OutROSTf and called inside #UpdateMessage.
     * This should be implemented in child class.
     *
     * @param OutROSTf
     */
    virtual void GetROS2Msg(FROSTFMsg& OutROSTf){};
};

/**
 * @brief TF Publisher class for single tf data. Please check #URRROS2OdomPublisher as example.
 * Please use #URRROS2TFsPublisher for multiple tf data.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFPublisher : public URRROS2TFPublisherBase
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform TF = FTransform::Identity;

    /**
     * @brief Set value to #TF.
     *
     * @param Translation
     * @param Rotation
     */
    UFUNCTION(BlueprintCallable)
    void SetTransform(const FVector& Translation, const FQuat& Rotation);

    /**
     * @brief Add tf to OutROSTf from #FrameId, #ChildFrameId and #TF
     *
     * @param OutROSTf
     */
    virtual void GetROS2Msg(FROSTFMsg& OutROSTf) override;
};

/**
 * @brief TF data component used in #URRROS2TFsPublisher
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFComponent : public UObject
{
    GENERATED_BODY()

public:
    /**
     * @brief Initialize TF data components with frame ids.
     *
     * @param InFrameId
     * @param InChildFrameId
     */
    UFUNCTION(BlueprintCallable)
    void Init(FString InFrameId, FString InChildFrameId)
    {
        FrameId = InFrameId;
        ChildFrameId = InChildFrameId;
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform TF = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ChildFrameId;

    /**
     * @brief return target FTransform.
     * This should be overrided in child class. Example is #URRROS2JointTFComponent
     *
     * @return FTransform
     */
    UFUNCTION(BlueprintCallable)
    virtual FTransform GetTF()
    {
        return TF;
    };
};

/**
 * @brief TF Publisher class for multiple tf data. Please check #URRRobotROS2Interface as example.
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2TFsPublisher : public URRROS2TFPublisherBase

{
    GENERATED_BODY()

public:
    //! TF data array
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<URRROS2TFComponent*> TFComponents;

    /**
     * @brief Add all #TFComponents to OutROSTf
     *
     * @param OutROSTf
     */
    virtual void GetROS2Msg(FROSTFMsg& OutROSTf) override;
};

/**
 * @brief TF data component for tf between links.
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2LinksTFComponent : public URRROS2TFComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Initialize with frame ids and initial transform between parent link and child link.
     *
     * @param InFrameId
     * @param InChildFrameId
     * @param InParentLink
     * @param InChildLink
     */
    UFUNCTION(BlueprintCallable)
    void InitLinksTFComponent(const FString& InFrameId,
                              const FString& InChildFrameId,
                              UPrimitiveComponent* InParentLink,
                              UPrimitiveComponent* InChildLink)
    {
        Super::Init(InFrameId, InChildFrameId);
        ParentLink = InParentLink;
        ChildLink = InChildLink;
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPrimitiveComponent* ParentLink = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UPrimitiveComponent* ChildLink = nullptr;

    /**
     * @brief return transform between child link and parent link.
     *
     * @return FTransform
     */
    virtual FTransform GetTF() override;

    /**
     * @brief Set Parent link and name
     *
     * @param InParentLink
     * @param InFrameId
     */
    UFUNCTION(BlueprintCallable)
    void AddParentLink(UPrimitiveComponent* InParentLink, const FString& InFrameId)
    {
        ParentLink = InParentLink;
        FrameId = InFrameId;
    }

    /**
     * @brief Set child link and name
     *
     * @param InChildLink
     * @param InChildFrameId
     */
    UFUNCTION(BlueprintCallable)
    void AddChildLink(UPrimitiveComponent* InChildLink, const FString& InChildFrameId)
    {
        ChildLink = InChildLink;
        ChildFrameId = InChildFrameId;
    }

    /**
     * @brief Add links from physics constraints
     *
     * @param InConstraint
     * @param InFrameId
     * @param InChildFrameId
     */
    UFUNCTION(BlueprintCallable)
    void AddLinksFromConstraint(UPhysicsConstraintComponent* InConstraint, const FString& InFrameId, const FString& InChildFrameId)
    {
        AddParentLink(URRGeneralUtils::GetPhysicsConstraintComponent(InConstraint, EConstraintFrame::Frame1), InFrameId);
        AddChildLink(URRGeneralUtils::GetPhysicsConstraintComponent(InConstraint, EConstraintFrame::Frame2), InChildFrameId);
    }

    /**
     * @brief Add URRROS2LinksTFComponent to #OutTFsPublisher::TFComponents.
     *
     * @param InParentLink
     * @param InChildLink
     * @param InFrameId
     * @param InChildFrameId
     * @param OutTFsPublisher
     */
    UFUNCTION(BlueprintCallable)
    static void AddLinks(UPrimitiveComponent* InParentLink,
                         UPrimitiveComponent* InChildLink,
                         const FString& InFrameId,
                         const FString& InChildFrameId,
                         URRROS2TFsPublisher* OutTFsPublisher);
};

/**
 * @brief TF data component for tf between child link and parent link which connected to the UPhysicsConstraintComponent.
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2PhysicsConstraintTFComponent : public URRROS2LinksTFComponent
{
    GENERATED_BODY()

public:
    /**
     @brief Initialize with frame ids and initial transform between parent link and joint and between joint to child link and target UPhysicsConstraintComponent.
     *
     * @param InFrameId
     * @param InChildFrameId
     * @param InConstraint
     */
    UFUNCTION(BlueprintCallable)
    void InitPhysicsConstraintTFComponent(const FString InFrameId,
                                          const FString InChildFrameId,
                                          UPhysicsConstraintComponent* InConstraint)
    {
        AddLinksFromConstraint(InConstraint, InFrameId, InChildFrameId);
    }

    /**
     * @brief Add URRROS2PhysicsConstraintTFComponent to #OutTFsPublisher::TFComponents.
     *
     * @param InConstraint
     * @param InFrameId
     * @param InChildFrameId
     * @param OutTFsPublisher
     */
    UFUNCTION(BlueprintCallable)
    static void AddConstraint(UPhysicsConstraintComponent* InConstraint,
                              const FString& InFrameId,
                              const FString& InChildFrameId,
                              URRROS2TFsPublisher* OutTFsPublisher);
};

/**
 * @brief TF data component for tf between child link and parent link which connected to the URRJointComponent.
 *
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2JointTFComponent : public URRROS2TFComponent
{
    GENERATED_BODY()

public:
    /**
     @brief Initialize with frame ids and initial transform between parent link and joint and between joint to child link and target URRJointComponent.
     *
     * @param InFrameId
     * @param InChildFrameId
     * @param InParentLinkToJoint
     * @param InJointToChildLink
     * @param InJoint
     */
    UFUNCTION(BlueprintCallable)
    void InitJointTFComponent(const FString& InFrameId, const FString& InChildFrameId, URRJointComponent* InJoint)
    {
        Super::Init(InFrameId, InChildFrameId);
        Joint = InJoint;
    }

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRJointComponent* Joint;

    virtual FTransform GetTF() override;

    /**
     * @brief Add URRROS2JointTFComponent to #OutTFsPublisher::TFComponents.
     *
     * @param InJoint
     * @param InFrameId
     * @param InChildFrameId
     * @param OutTFsPublisher
     */
    UFUNCTION(BlueprintCallable)
    static void AddJoint(URRJointComponent* InJoint,
                         const FString& InFrameId,
                         const FString& InChildFrameId,
                         URRROS2TFsPublisher* OutTFsPublisher);
};
