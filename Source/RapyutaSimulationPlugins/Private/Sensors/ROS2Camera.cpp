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
    Super::Init();

    // component initialization
    SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D::StaticClass());
    // SceneCaptureComponent->RegisterComponent(); // not required?
    RenderTarget = NewObject<UTextureRenderTarget2D>(this, UTextureRenderTarget2D::StaticClass());

    // Initialize capture and texture component
    SceneCaptureComponent->SetWorldTransform(this->GetTransform());
    SceneCaptureComponent->FOVAngle = FOV;
    SceneCaptureComponent->OrthoWidth = OrthWidth;
    
    RenderTarget->InitCustomFormat(Width, Height, EPixelFormat::PF_B8G8R8A8, true);
    SceneCaptureComponent->TextureTarget = RenderTarget;

    // Initialize image data
    Data.width = Width;
    Data.height = Height;
    Data.encoding = Encoding;
    Data.step = Width * 3; // todo should be variable based on encoding
    Data.data.AddUninitialized(Width * Height * 3);

    // ROS Image topic publisher
    Publisher =  NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
    Publisher->RegisterComponent();
    Publisher->TopicName = TopicName;
    Publisher->PublicationFrequencyHz = PublishFreq;
    Publisher->MsgClass = UROS2ImageMsg::StaticClass();
    
    Publisher->UpdateDelegate.BindDynamic(this, &AROS2Camera::MessageUpdate);
    AddPublisher(Publisher);
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

    FTextureRenderTarget2DResource* RenderTargetResource;
    RenderTargetResource = (FTextureRenderTarget2DResource*)RenderTarget->GameThread_GetRenderTargetResource();
    if (RenderTargetResource) {
        TArray<FColor> buffer;
        RenderTargetResource->ReadPixels(buffer);
        for (int I = 0; I < buffer.Num(); I++)
        {
            Data.data[I * 3 + 0] = buffer[I].R;
            Data.data[I * 3 + 1] = buffer[I].G;
            Data.data[I * 3 + 2] = buffer[I].B;
        }
    }
    
    return Data;
}