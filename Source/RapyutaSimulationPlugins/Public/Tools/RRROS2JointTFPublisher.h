/**
 * @file RRROS2ActorTFPublisher.h
 * @brief Publish TF of Actor relative to the reference actor.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "CoreMinimal.h"
#include "Kismet/GameplayStatics.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2TFPublisher.h"

#include "RRROS2JointTFPublisher.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2JointTFComponentBase : public URRROS2TFComponent
{
    GENERATED_BODY()

public:
    URRROS2JointTFComponentBase(){};

    URRROS2JointTFComponentBase(FString InFrameId,
                                FString InChildFrameId,
                                FTransform InParentLinkToJoint,
                                FTransform InJointToChildLink)
        : URRROS2TFComponent(InFrameId, InChildFrameId),
          ParentLinkToJoint(InParentLinkToJoint),
          JointToChildLink(InJointToChildLink){};

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform ParentLinkToJoint = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FTransform JointToChildLink = FTransform::Identity;

    virtual FTransform GetTF() override;
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2JointTFComponent : public URRROS2JointTFComponentBase
{
    GENERATED_BODY()

public:
    URRROS2JointTFComponent(){};

    URRROS2JointTFComponent(FString InFrameId,
                            FString InChildFrameId,
                            FTransform InParentLinkToJoint,
                            FTransform InJointToChildLink)
        : URRROS2JointTFComponentBase(InFrameId, InChildFrameId, InParentLinkToJoint, InJointToChildLink){};

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRJointComponent* Joint;

    virtual FTransform GetTF() override;
};

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2JointsTFPublisher : public URRROS2TFsPublisher

{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable)
    virtual void AddJoint(URRJointComponent* InJoint, FString InFrameId, FString InChildFrameId);
};
