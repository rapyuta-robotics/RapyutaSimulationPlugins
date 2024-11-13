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
    if (!OrientationNoise)
    {
        OrientationNoise = NewObject<URRGaussianNoise>(this, *FString::Printf(TEXT("%sOrientationNoise"), *GetName()));
    }
    if (!AngularVelocityNoise)
    {
        AngularVelocityNoise = NewObject<URRGaussianNoise>(this, *FString::Printf(TEXT("%sAngularVelocityNoise"), *GetName()));
    }
    LinearAccelerationNoise->Init(NoiseMeanLinearAcceleration, NoiseVarianceLinearAcceleration);
    OrientationNoise->Init(NoiseMeanOrientation, NoiseVarianceOrientation);
    AngularVelocityNoise->Init(NoiseMeanAngularVelocity, NoiseVarianceAngularVelocity);

    Data.OrientationCovariance[0] = Data.OrientationCovariance[4] = Data.OrientationCovariance[8] = OrientationNoise->Cov;
    Data.AngularVelocityCovariance[0] = Data.AngularVelocityCovariance[4] = Data.AngularVelocityCovariance[8] =
        AngularVelocityNoise->Cov;
    Data.LinearAccelerationCovariance[0] = Data.LinearAccelerationCovariance[4] = Data.LinearAccelerationCovariance[8] =
        LinearAccelerationNoise->Cov;
}

FROSImu URRROS2IMUComponent::GetROS2Data()
{
    return Data;
}

void URRROS2IMUComponent::SensorUpdate()
{
    const float currentTime = UGameplayStatics::GetTimeSeconds(GetWorld());
    const FTransform worldTransform = K2_GetComponentToWorld();
    const FTransform relTransform = worldTransform * InitialTransform.Inverse();
    const float dt = currentTime - LastSensorUpdateTime;

    if (dt > 1e-10)
    {
        const float _dt = 1.0 / dt;
        Data.Header.FrameId = FrameId;
        Data.Header.Stamp = URRConversionUtils::FloatToROSStamp(currentTime);

        // Transform difference in reference(initial) transform
        const FTransform dT = relTransform * LastTransform.Inverse();

        FQuat orientation = relTransform.GetRotation();    // Rotation from reference(initial) transform
        FVector angularVelocity =
            orientation.UnrotateVector(dT.GetRotation().GetNormalized().Euler() * _dt);    //in sensor local transform

        const FVector linearVel = dT.GetTranslation() * _dt;
        FVector linearAcc = orientation.UnrotateVector((linearVel - LastLinearVel) * _dt);    //in sensor local transform

        // post process, add gravity and scale
        if (bAddGravity)
        {
            FVector GravityAcc =
                FVector(0.0, 0.0, -UnitConversion::ForceUnificationFactor(EUnit::KilogramsForce) * 100);    // cm/ss
            linearAcc += worldTransform.GetRotation().UnrotateVector(GravityAcc);
        }
        linearAcc *= AccGain;

        // noise
        LinearAcceleration =
            linearAcc + FVector(LinearAccelerationNoise->Get(), LinearAccelerationNoise->Get(), LinearAccelerationNoise->Get());
        AngularVelocity =
            angularVelocity + FVector(AngularVelocityNoise->Get(), AngularVelocityNoise->Get(), AngularVelocityNoise->Get());
        OrientationNoiseSum = OrientationNoiseSum + OrientationNoiseDriftCoefficient * OrientationNoise->Get();
        Orientation = FQuat::MakeFromEuler(orientation.Euler() + OrientationNoiseSum + OffsetOrientation);

        // UE -> ROS conversion
        Data.LinearAcceleration = URRConversionUtils::VectorUEToROS(LinearAcceleration);
        Data.AngularVelocity = URRConversionUtils::VectorUEToROS(AngularVelocity);
        Data.Orientation = URRConversionUtils::QuatUEToROS(Orientation);

        // update
        LastTransform = relTransform;
        LastdT = dT;
        LastLinearVel = linearVel;
        LastSensorUpdateTime = currentTime;
        bIsValid = true;
    }
    else
    {
        UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Warning, TEXT("Sensor update ratio is too fast: %f"), dt);
    }
}

void URRROS2IMUComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2ImuMsg>(InMessage)->SetMsg(GetROS2Data());
}
