// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/ROS2Camera.h"

AROS2Camera::AROS2Camera()
{
}

void AROS2Camera::BeginPlay()
{
	Super::BeginPlay();	
    Init();
}

void AROS2Camera::Init()
{
    // component initialization
    SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D::StaticClass());
    // SceneCaptureComponent->RegisterComponent(); // not required?
    RenderTarget = NewObject<UTextureRenderTarget2D>(this, UTextureRenderTarget2D::StaticClass());

    // Initialize capture and texture component
    SceneCaptureComponent->SetWorldTransform(this->GetTransform());
    SceneCaptureComponent->FOVAngle = GetCameraComponent()->FieldOfView;
    SceneCaptureComponent->OrthoWidth = GetCameraComponent()->OrthoWidth;
    
    RenderTarget->InitCustomFormat(Width, Height, EPixelFormat::PF_B8G8R8A8, true);
    SceneCaptureComponent->TextureTarget = RenderTarget;

    // Initialize image data
    Data.width = Width;
    Data.height = Height;
    Data.encoding = Encoding;
    Data.step = Width * 3; // todo should be variable based on encoding
    Data.data.AddUninitialized(Width * Height * 3);

    // Node and publisher initialize
    FActorSpawnParameters SpawnParamsNode;
    Node = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
    Node->Name = NodeName.IsEmpty() ? TEXT("UE4CameraNode_" + FGuid::NewGuid().ToString()) : NodeName;
    Node->Namespace = FString();
    Node->Init();

    Publisher =  NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
    Publisher->RegisterComponent();
    Publisher->TopicName = TopicName;
    Publisher->PublicationFrequencyHz = PublishFreq;
    Publisher->MsgClass = UROS2ImageMsg::StaticClass();
    
    Publisher->UpdateDelegate.BindDynamic(this, &AROS2Camera::MessageUpdate);
    Node->AddPublisher(Publisher);
    Publisher->Init(UROS2QoS::KeepLast);

}

void AROS2Camera::MessageUpdate(UROS2GenericMsg *TopicMessage)
{
    UROS2ImageMsg *Message = Cast<UROS2ImageMsg>(TopicMessage);
    Message->SetMsg(GetData());
}

FROSImage AROS2Camera::GetData()
{
    SceneCaptureComponent->CaptureScene();
    CaptureNonBlocking();
    if(!RenderRequestQueue.IsEmpty()){
        // Peek the next RenderRequest from queue
        FRenderRequest* nextRenderRequest = nullptr;
        RenderRequestQueue.Peek(nextRenderRequest);

        if(nextRenderRequest){ //nullptr check
            if(nextRenderRequest->RenderFence.IsFenceComplete()){ // Check if rendering is done, indicated by RenderFence
                for (int I = 0; I < nextRenderRequest->Image.Num(); I++)
                {
                    Data.data[I * 3 + 0] = nextRenderRequest->Image[I].R;
                    Data.data[I * 3 + 1] = nextRenderRequest->Image[I].G;
                    Data.data[I * 3 + 2] = nextRenderRequest->Image[I].B;
                    // UE_LOG(LogTemp, Warning, TEXT("AsyncTaskDone"));
                }

                // Delete the first element from RenderQueue
                RenderRequestQueue.Pop();
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

// reference https://github.com/TimmHess/UnrealImageCapture
void AROS2Camera::CaptureNonBlocking(){

   SceneCaptureComponent->TextureTarget->TargetGamma = GEngine->GetDisplayGamma();
    // Get RenderContext
    FTextureRenderTargetResource* renderTargetResource = SceneCaptureComponent->TextureTarget->GameThread_GetRenderTargetResource();

    struct FReadSurfaceContext{
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
        FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)
    };

    // Above 4.22 use this
    ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
    [readSurfaceContext, this](FRHICommandListImmediate& RHICmdList){
        RHICmdList.ReadSurfaceData(
            readSurfaceContext.SrcRenderTarget->GetRenderTargetTexture(),
            readSurfaceContext.Rect,
            *readSurfaceContext.OutData,
            readSurfaceContext.Flags
        );
    });

    // Notify new task in RenderQueue
    RenderRequestQueue.Enqueue(renderRequest);

    // Set RenderCommandFence
    renderRequest->RenderFence.BeginFence();
}