// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "Msgs/ROS2PointCloud2Msg.h"
#include "Sensors/BaseLidar.h"

#include "SensorLidar3D.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ASensorLidar3D : public ABaseLidar
{
	GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    ASensorLidar3D();

    // Called every frame
    virtual void Tick(float DeltaTime) override;
    
    void Run() override;

    void Scan() override;

    void LidarMessageUpdate(UROS2GenericMsg* TopicMessage) override;
    
    bool Visible(AActor* TargetActor) override;

    void InitLidar(AROS2Node* Node, const FString& TopicName) override;

    void InitToNode(AROS2Node* Node) override;

    // this should probably be removed so that the sensor can be decoupled from the message types
    FROSPointCloud2 GetROS2Data();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NChannelsPerScan = 0; // vertical samples
	
    // [degrees]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartVerticalAngle = 0.f;
	
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float FOVVertical = 0.f;

    // [degrees]
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float DVAngle = 0.f;
};
