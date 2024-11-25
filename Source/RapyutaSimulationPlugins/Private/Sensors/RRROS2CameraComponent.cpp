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

    if (CameraType == EROS2CameraType::DEPTH)
    {
        FWeightedBlendable blendable(1.0f, GetBufferVisualizationData().GetMaterial(TEXT("SceneDepth")));
        CameraComponent->PostProcessSettings.WeightedBlendables.Array.Add(blendable);
        SceneCaptureComponent->PostProcessSettings = CameraComponent->PostProcessSettings;
        SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorHDR;
    }

    RenderTarget = NewObject<UTextureRenderTarget2D>(this, UTextureRenderTarget2D::StaticClass());
    RenderTarget->InitCustomFormat(Width, Height, EPixelFormat::PF_B8G8R8A8, true);
    SceneCaptureComponent->TextureTarget = RenderTarget;

    // Initialize image data
    Data.Header.FrameId = FrameId;
    Data.Width = Width;
    Data.Height = Height;
    Data.Encoding = Encoding;
    Data.Step = Width * 3;    // todo should be variable based on encoding
    Data.Data.AddUninitialized(Width * Height * 3);

    QueueSize = QueueSize < 1 ? 1 : QueueSize;    // QueueSize should be more than 1

    Super::PreInitializePublisher(InROS2Node, InTopicName);
}

void URRROS2CameraComponent::SensorUpdate()
{
    if (Render) {
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

    struct FReadSurfaceContext
    {
        FRenderTarget* SrcRenderTarget;
        TArray<FColor>* OutData;
        FIntRect Rect;
        FReadSurfaceDataFlags Flags;
    };

    // Init new RenderRequest
    FRenderRequest* renderRequest = new FRenderRequest();

    // Setup GPU command
    FReadSurfaceContext readSurfaceContext = {
        renderTargetResource,
        &(renderRequest->Image),
        FIntRect(0, 0, renderTargetResource->GetSizeXY().X, renderTargetResource->GetSizeXY().Y),
        FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Above 4.22 use this
    ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
    (
        [readSurfaceContext, this](FRHICommandListImmediate& RHICmdList)
        {
            RHICmdList.ReadSurfaceData(readSurfaceContext.SrcRenderTarget->GetRenderTargetTexture(),
                                       readSurfaceContext.Rect,
                                       *readSurfaceContext.OutData,
                                       readSurfaceContext.Flags);
        });

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
        if (nextRenderRequest)
        {    // nullptr check
            if (nextRenderRequest->RenderFence.IsFenceComplete())
            {    // Check if rendering is done, indicated by RenderFence
                for (int I = 0; I < nextRenderRequest->Image.Num(); I++)
                {
                    Data.Data[I * 3 + 0] = nextRenderRequest->Image[I].R;
                    Data.Data[I * 3 + 1] = nextRenderRequest->Image[I].G;
                    Data.Data[I * 3 + 2] = nextRenderRequest->Image[I].B;
                }

                // Delete the first element from RenderQueue
                RenderRequestQueue.Pop();
                QueueCount--;
                delete nextRenderRequest;
            }
        }
    }

    // SceneCaptureComponent->CaptureScene();
    // FTextureRenderTarget2DResource* RenderTargetResource;
    // RenderTargetResource = (FTextureRenderTarget2DResource*)RenderTarget->GameThread_GetRenderTargetResource();
    // if (RenderTargetResource) {
    //     TArray<FColor> buffer;
    //     RenderTargetResource->ReadPixels(buffer);
    //     for (int I = 0; I < buffer.Num(); I++)
    //     {
    //         Data.data[I * 3 + 0] = buffer[I].R;
    //         Data.data[I * 3 + 1] = buffer[I].G;
    //         Data.data[I * 3 + 2] = buffer[I].B;
    //     }
    // }

    return Data;
}

void URRROS2CameraComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2ImgMsg>(InMessage)->SetMsg(GetROS2Data());
}
