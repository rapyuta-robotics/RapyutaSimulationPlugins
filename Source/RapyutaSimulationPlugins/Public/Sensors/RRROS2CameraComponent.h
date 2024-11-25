/**
 * @file RRROS2CameraComponent.h
 * @brief ROS 2 Camera component
 * @todo Support non RGB data support.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"

// rclUE
#include <Msgs/ROS2Img.h>

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "RRROS2BaseSensorComponent.h"

#include "RRROS2CameraComponent.generated.h"

/**
 * @brief
 * used in　#CaptureNonBlocking of #URRROS2CameraComponent
 */
USTRUCT()
struct FRenderRequest
{
    GENERATED_BODY()
    TArray<FColor> Image;
    FRenderCommandFence RenderFence;
};

UENUM(BlueprintType)
enum class EROS2CameraType : uint8
{
    RGB UMETA(DisplayName = "RGB"),
    DEPTH UMETA(DisplayName = "Depth")
};

/**
 * @brief ROS 2 Camera component. Uses USceneCaptureComponent2D.
 *
 * @sa [USceneCaptureComponent2D](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Components/USceneCaptureComponent2D/)
 * @sa [UE4 ShaderInPlugin](https://docs.unrealengine.com/5.1/en-US/ProgrammingAndScripting/Rendering/ShaderInPlugin/Overview/)
 * @sa implementation reference: https://github.com/TimmHess/UnrealImageCapture
 * @todo Support non RGB data support.
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRROS2CameraComponent : public URRROS2BaseSensorComponent
{
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new URRROS2CameraComponent object
     * Initialize #SceneCaptureComponent and #CameraComponent.
     */
    URRROS2CameraComponent();

    /**
     * @brief Initialize #Data and #RenderTarget, set #SceneCaptureComponent parameters.
     *
     * @param InROS2Node ROS2Node which this publisher belongs to
     * @param InTopicName
     */
    virtual void PreInitializePublisher(UROS2NodeComponent* InROS2Node, const FString& InTopicName) override;

    /**
     * @brief Update sensor data by CaptureScene and #CaptureNonBlocking
     * @sa [CaptureScene](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Components/USceneCaptureComponent2D/CaptureScene/)
     * @todo Should #CaptureNonBlocking called in TickComponents?
     */
    virtual void SensorUpdate() override;

protected:
    /**
     * @brief Capture data by notifing task in #RenderRequestQueue
     * @sa reference https://github.com/TimmHess/UnrealImageCapture
     */
    UFUNCTION()
    void CaptureNonBlocking();

    //!
    TQueue<FRenderRequest*> RenderRequestQueue;

    //!
    FROSImg Data;

    int32 QueueCount = 0;

public:
    //! Camera. Not necessary to capture but useful to see image in UE4 windows.
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    UCameraComponent* CameraComponent = nullptr;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    USceneCaptureComponent2D* SceneCaptureComponent = nullptr;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
    UTextureRenderTarget2D* RenderTarget = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 Width = 640;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 Height = 480;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 QueueSize = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    EROS2CameraType CameraType = EROS2CameraType::RGB;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool Render = true;
   
    // ROS
    /**
     * @brief Update ROS 2 Msg structure from #RenderRequestQueue
     *
     * @return FROSImg
     */
    UFUNCTION(BlueprintCallable)
    virtual FROSImg GetROS2Data();

    /**
     * @brief Set result of #GetROS2Data to InMessage.
     *
     * @param InMessage
     */
    virtual void SetROS2Msg(UROS2GenericMsg* InMessage) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Encoding = TEXT("rgb8");
};
