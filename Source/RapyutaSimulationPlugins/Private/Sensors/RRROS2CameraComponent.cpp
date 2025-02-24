// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2CameraComponent.h"

#include "BufferVisualizationData.h"

URRROS2CameraComponent::URRROS2CameraComponent()
{
    // component initialization
    SceneCaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCaptureComponent"));
    SceneCaptureComponent->SetupAttachment(this);

    CameraComponent = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraComponent"));
    CameraComponent->SetupAttachment(this);

    TopicName = TEXT("raw_image");
    MsgClass = UROS2ImgMsg::StaticClass();
}

void URRROS2CameraComponent::PreInitializePublisher(UROS2NodeComponent* InROS2Node, const FString& InTopicName)
{
    SceneCaptureComponent->FOVAngle = CameraComponent->FieldOfView;
    SceneCaptureComponent->OrthoWidth = CameraComponent->OrthoWidth;

    // Set capture source and image properties based on camera type
    if (CameraType == EROS2CameraType::RGB)
    {
        SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorHDR;
        RenderTarget = NewObject<UTextureRenderTarget2D>(this, UTextureRenderTarget2D::StaticClass());
        RenderTarget->InitCustomFormat(Width, Height, EPixelFormat::PF_B8G8R8A8, true);

        Data.Encoding = TEXT("bgr8");
        Data.Step = Width * 3;
    }
    else if (CameraType == EROS2CameraType::DEPTH)
    {
        CameraComponent->PostProcessSettings.WeightedBlendables.Array.Add(
            FWeightedBlendable(1.0f, GetBufferVisualizationData().GetMaterial(TEXT("SceneDepth"))));
        SceneCaptureComponent->PostProcessSettings = CameraComponent->PostProcessSettings;
        SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;

        RenderTarget = NewObject<UTextureRenderTarget2D>(this, UTextureRenderTarget2D::StaticClass());
        RenderTarget->InitCustomFormat(Width, Height, EPixelFormat::PF_FloatRGBA, false);
        Data.Encoding = TEXT("32FC1");
        Data.Step = Width * 4;
    }

    // Common setup
    SceneCaptureComponent->TextureTarget = RenderTarget;
    Data.Header.FrameId = FrameId;
    Data.Width = Width;
    Data.Height = Height;
    Data.Data.AddUninitialized(Width * Height * (CameraType == EROS2CameraType::RGB ? 3 : 4));
    QueueSize = QueueSize < 1 ? 1 : QueueSize;    // QueueSize should be more than 1
    Super::PreInitializePublisher(InROS2Node, InTopicName);
}

void URRROS2CameraComponent::SensorUpdate()
{
    if (Render)
    {
        SceneCaptureComponent->CaptureScene();
        CaptureNonBlocking();
    }
}

// reference https://github.com/TimmHess/UnrealImageCapture
void URRROS2CameraComponent::CaptureNonBlocking()
{
    SceneCaptureComponent->TextureTarget->TargetGamma = GEngine->GetDisplayGamma();
    // Get RenderContext
    FTextureRenderTargetResource* renderTargetResource = SceneCaptureComponent->TextureTarget->GameThread_GetRenderTargetResource();

    struct FReadSurfaceContextRGB
    {
        FRenderTarget* SrcRenderTarget;
        TArray<FColor>* OutData;
        FIntRect Rect;
        FReadSurfaceDataFlags Flags;
    };

    struct FReadSurfaceContextDepth
    {
        FRenderTarget* SrcRenderTarget;
        TArray<FFloat16Color>* Depth;
        FIntRect Rect;
        FReadSurfaceDataFlags Flags;
    };

    // Init new RenderRequest
    FRenderRequest* renderRequest = new FRenderRequest();

    // Setup GPU command
    FIntRect rect(0, 0, renderTargetResource->GetSizeXY().X, renderTargetResource->GetSizeXY().Y);
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);

    if (CameraType == EROS2CameraType::RGB)
    {
        FReadSurfaceContextRGB readSurfaceContext = {renderTargetResource, &(renderRequest->Image), rect, flags};

        ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
        (
            [readSurfaceContext, this](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.ReadSurfaceData(readSurfaceContext.SrcRenderTarget->GetRenderTargetTexture(),
                                           readSurfaceContext.Rect,
                                           *readSurfaceContext.OutData,
                                           readSurfaceContext.Flags);
            });
    }
    else if (CameraType == EROS2CameraType::DEPTH)
    {
        FReadSurfaceContextDepth readSurfaceContext = {renderTargetResource, &(renderRequest->Depth), rect, flags};

        ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
        (
            [readSurfaceContext, this](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.ReadSurfaceFloatData(readSurfaceContext.SrcRenderTarget->GetRenderTargetTexture(),
                                                readSurfaceContext.Rect,
                                                *readSurfaceContext.Depth,
                                                readSurfaceContext.Flags);
            });
    }

    // Notify new task in RenderQueue
    RenderRequestQueue.Enqueue(renderRequest);
    if (QueueCount > QueueSize)
    {
        RenderRequestQueue.Pop();
    }
    else
    {
        QueueCount++;
    }

    // Set RenderCommandFence
    renderRequest->RenderFence.BeginFence();
}

FROSImg URRROS2CameraComponent::GetROS2Data()
{
    if (!RenderRequestQueue.IsEmpty())
    {
        // Timestamp
        Data.Header.Stamp = URRConversionUtils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(GetWorld()));
        // Peek the next RenderRequest from queue
        FRenderRequest* nextRenderRequest = nullptr;
        RenderRequestQueue.Peek(nextRenderRequest);
        if (nextRenderRequest && nextRenderRequest->RenderFence.IsFenceComplete())
        {
            if (CameraType == EROS2CameraType::RGB)
            {
                // Process RGB data
                for (int i = 0; i < nextRenderRequest->Image.Num(); i++)
                {
                    Data.Data[i * 3 + 0] = nextRenderRequest->Image[i].B;
                    Data.Data[i * 3 + 1] = nextRenderRequest->Image[i].G;
                    Data.Data[i * 3 + 2] = nextRenderRequest->Image[i].R;
                }
            }
            else if (CameraType == EROS2CameraType::DEPTH)
            {
                // Process Depth data
                for (int i = 0; i < nextRenderRequest->Depth.Num(); i++)
                {
                    float value = nextRenderRequest->Depth[i].R.GetFloat() / 100;
                    std::memcpy(&Data.Data[i * 4], &value, sizeof(value));
                }
            }

            // Delete the first element from RenderQueue
            RenderRequestQueue.Pop();
            QueueCount--;
            delete nextRenderRequest;
        }
    }
    return Data;
}

void URRROS2CameraComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2ImgMsg>(InMessage)->SetMsg(GetROS2Data());
}
