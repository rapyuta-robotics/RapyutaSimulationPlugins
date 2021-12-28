// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "Msgs/ROS2PointCloud2Msg.h"

// RapyutaSimulationPlugins
#include "Sensors/RRBaseLidarComponent.h"

#include "RR3DLidarComponent.generated.h"

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URR3DLidarComponent : public URRBaseLidarComponent
{
    GENERATED_BODY()

public:
    URR3DLidarComponent();
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    void Run() override;

    void Scan() override;

    bool Visible(AActor* TargetActor) override;

    // (ToDo) this should probably be removed so that the sensor can be decoupled from the message types
    FROSPointCloud2 GetROS2Data();

    // vertical samples
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NChannelsPerScan = 32;

    uint64 GetTotalScan() const
    {
        return NSamplesPerScan * NChannelsPerScan;
    }

    // [degrees]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartVerticalAngle = -45.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float FOVVertical = 90.f;

    // [degrees]
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float DVAngle = 0.f;
};
