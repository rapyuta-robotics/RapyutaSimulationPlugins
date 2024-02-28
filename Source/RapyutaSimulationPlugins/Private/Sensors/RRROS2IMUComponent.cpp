// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRROS2IMUComponent.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRUObjectUtils.h"

URRROS2IMUComponent::URRROS2IMUComponent()
{
    TopicName = TEXT("imu");
    MsgClass = UROS2ImuMsg::StaticClass();
}

void URRROS2IMUComponent::BeginPlay()
{
    Super::BeginPlay();
    Reset();
}

void URRROS2IMUComponent::Reset()
{
    InitialTransform = K2_GetComponentToWorld();
    LastSensorUpdateTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    OrientationNoiseSum = FVector::ZeroVector;

    // noise
    if (!LinearAccelerationNoise)
    {
        LinearAccelerationNoise =
            NewObject<URRGaussianNoise>(this, *FString::Printf(TEXT("%sLinearAccelerationNoise"), *GetName()));
    }
    LinearAccelerationNoise->Init(NoiseMeanLinearAcceleration, NoiseVarianceLinearAcceleration);
    OrientationNoise->Init(NoiseMeanOrientation, NoiseVarianceOrientation);
    AngularVelocityNoise->Init(NoiseMeanAngularVelocity, NoiseVarianceAngularVelocity);
}

FROSImu URRROS2IMUComponent::GetROS2Data()
{
    return Data;
}

void URRROS2IMUComponent::SensorUpdate()
{
    const float currentTime = UGameplayStatics::GetTimeSeconds(GetWorld());
    const FTransform currentTransform = K2_GetComponentToWorld();
    const float dt = 1.0 / (currentTime - LastSensorUpdateTime);

    if (dt < 1e-10)
    {
        const float _dt = 1.0 / dt;
        Data.Header.Stamp = URRConversionUtils::FloatToROSStamp(currentTime);

        const FTransform dT = currentTransform * InitialTransform.Inverse();
        const FTransform d2T = dT * LastdT.Inverse();

        FQuat orientation = currentTransform.GetRotation();
        FVector angularVelocity = dT.GetRotation().GetNormalized().Euler() * _dt;
        FVector linearAcc = d2T.GetTranslation() * _dt * _dt;

        // noise
        Data.LinearAcceleration =
            linearAcc + FVector(LinearAccelerationNoise->Get(), LinearAccelerationNoise->Get(), LinearAccelerationNoise->Get());
        Data.AngularVelocity =
            angularVelocity + FVector(AngularVelocityNoise->Get(), AngularVelocityNoise->Get(), AngularVelocityNoise->Get());
        // OrientationNoiseSum +=  OrientationNoiseDriftCoefficient  OrientationNoise->Get();
        Data.Orientation = FQuat::MakeFromEuler(orientation.Euler() + OrientationNoiseSum + OffsetOrientation);

        LastTransform = currentTransform;
        LastdT = dT;
        LastSensorUpdateTime = currentTime;
        bIsValid = true;
    }
    else
    {
        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Warning, TEXT("Sensor update ratio is too fast."));
    }
}

void URRROS2IMUComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2ImuMsg>(InMessage)->SetMsg(GetROS2Data());
}
