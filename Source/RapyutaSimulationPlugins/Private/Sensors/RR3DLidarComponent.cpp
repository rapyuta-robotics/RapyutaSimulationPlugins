// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RR3DLidarComponent.h"

// UE
#include "Async/ParallelFor.h"
// rclUE
#include "Msgs/ROS2PointField.h"
#include "rclcUtilities.h"

URR3DLidarComponent::URR3DLidarComponent()
{
    TopicName = TEXT("scan");
    MsgClass = UROS2PointCloud2Msg::StaticClass();
}
void URR3DLidarComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
#if TRACE_ASYNC
    verify(TraceHandles.Num() == RecordedHits.Num());
    UWorld* world = GetWorld();
    for (auto i = 0; i < TraceHandles.Num(); ++i)
    {
        FTraceHandle& traceHandle = TraceHandles[i];
        FHitResult& recordedHit = RecordedHits[i];
        if (traceHandle._Data.FrameNumber != 0)
        {
            FTraceDatum Output;
            if (world->QueryTraceData(traceHandle, Output))
            {
                if (Output.OutHits.Num() > 0)
                {
                    traceHandle._Data.FrameNumber = 0;
                    // We should only be tracing the first hit anyhow
                    recordedHit = Output.OutHits[0];
                }
                else
                {
                    traceHandle._Data.FrameNumber = 0;
                    recordedHit = FHitResult();
                    recordedHit.TraceStart = Output.Start;
                    recordedHit.TraceEnd = Output.End;
                }
            }
        }
    }
#endif
}

void URR3DLidarComponent::Run()
{
    const uint64 nTotalScan = GetTotalScan();

    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);
    DVAngle = FOVVertical / static_cast<float>(NChannelsPerScan);

    RecordedHits.Init(FHitResult(ForceInit), nTotalScan);

#if TRACE_ASYNC
    TraceHandles.Init(FTraceHandle(), nTotalScan);
#endif
    Super::Run();
}

void URR3DLidarComponent::SensorUpdate()
{
    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("3DLaser_Trace"), true);
    TraceParams.bReturnPhysicalMaterial = true;

    // TraceParams.bIgnoreTouches = true;
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnFaceIndex = true;

    if (bIgnoreSelf)
    {
        TraceParams.AddIgnoredActor(GetOwner());
    }

    FVector lidarPos = GetComponentLocation();
    FRotator lidarRot = GetComponentRotation();

#if TRACE_ASYNC
    // This is cheesy, but basically if the first trace is in flight we assume they're all waiting and don't do another trace.
    // This is not good if done on other threads and only works because both timers and actor ticks happen on the game thread.
    if ((TraceHandles.Num() > 0) && (TraceHandles[0]._Data.FrameNumber == 0))
    {
        UWorld* world = GetWorld();
        for (auto i = 0; i < TraceHandles.Num(); ++i)
        {
            const int IdxX = i % NSamplesPerScan;
            const int IdxY = i / NSamplesPerScan;
            const float HAngle = StartAngle + DHAngle * IdxX;
            const float VAngle = StartVerticalAngle + DVAngle * IdxY;

            FRotator laserRot(VAngle, HAngle, 0);
            FRotator rot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

            FVector startPos = lidarPos + MinRange * UKismetMathLibrary::GetForwardVector(rot);
            FVector endPos = lidarPos + MaxRange * UKismetMathLibrary::GetForwardVector(rot);
            // To be considered: += WithNoise * FVector(PositionNoise->Get(),PositionNoise->Get(),PositionNoise->Get());

            TraceHandles[i] = world->AsyncLineTraceByChannel(EAsyncTraceType::Single,
                                                             startPos,
                                                             endPos,
                                                             TraceCollisionChannel,
                                                             TraceParams,
                                                             FCollisionResponseParams::DefaultResponseParam,
                                                             nullptr);
        }
    }
#else
    ParallelFor(
        RecordedHits.Num(),
        [this, &TraceParams, &lidarPos, &lidarRot](int32 Index)
        {
            const int IdxX = Index % NSamplesPerScan;
            const int IdxY = Index / NSamplesPerScan;
            const float HAngle = StartAngle + DHAngle * IdxX;
            const float VAngle = StartVerticalAngle + DVAngle * IdxY;

            FRotator laserRot(VAngle, HAngle, 0);
            FRotator rot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

            FVector startPos = lidarPos + MinRange * UKismetMathLibrary::GetForwardVector(rot);
            FVector endPos = lidarPos + MaxRange * UKismetMathLibrary::GetForwardVector(rot);
            // + WithNoise *  FVector(PositionNoise->Get(),PositionNoise->Get(),PositionNoise->Get());

            GetWorld()->LineTraceSingleByChannel(RecordedHits[Index],
                                                 startPos,
                                                 endPos,
                                                 TraceCollisionChannel,
                                                 TraceParams,
                                                 FCollisionResponseParams::DefaultResponseParam);
        },
        false);
#endif

    if (BWithNoise)
    {
        // this approach to noise is different from the above:
        // noise on the linetrace input means that the further the hit, the larger the error, while here the error is independent
        // from distance
        ParallelFor(
            RecordedHits.Num(),
            [this, &TraceParams, &lidarPos, &lidarRot](int32 Index)
            {
                RecordedHits[Index].ImpactPoint += FVector(PositionNoise->Get(), PositionNoise->Get(), PositionNoise->Get());
                RecordedHits[Index].TraceEnd += FVector(PositionNoise->Get(), PositionNoise->Get(), PositionNoise->Get());
            },
            false);
    }

    TimeOfLastScan = UGameplayStatics::GetTimeSeconds(GetWorld());
    Dt = 1.f / static_cast<float>(PublicationFrequencyHz);

    // need to store on a structure associating hits with time?
    // GetROS2Data needs to get all data since the last Get? or the last within the last time interval?

    ULineBatchComponent* const LineBatcher = GetWorld()->PersistentLineBatcher;
    if (LineBatcher != nullptr && bShowLidarRays)
    {
        for (auto& h : RecordedHits)
        {
            if (h.GetActor() != nullptr)
            {
                float Distance = (MinRange * (h.Distance > 0) + h.Distance) * .01f;
                if (h.PhysMaterial != nullptr)
                {
                    // retroreflective material
                    if (h.PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType1)
                    {
                        // UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("retroreflective surface type hit"));
                        // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorReflected, 10, .5, dt);
                        LineBatcher->DrawPoint(h.ImpactPoint,
                                               InterpColorFromIntensity(GetIntensityFromDist(IntensityReflective, Distance)),
                                               5,
                                               DrawPointDepthIntensity,
                                               Dt);
                    }
                    // non reflective material
                    else if (h.PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType_Default)
                    {
                        // UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("default surface type hit"));
                        // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorHit, 10, .5, dt);
                        LineBatcher->DrawPoint(h.ImpactPoint,
                                               InterpColorFromIntensity(GetIntensityFromDist(IntensityNonReflective, Distance)),
                                               5,
                                               DrawPointDepthIntensity,
                                               Dt);
                    }
                    // reflective material
                    else if (h.PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType2)
                    {
                        FVector HitSurfaceNormal = h.Normal;
                        FVector RayDirection = h.TraceEnd - h.TraceStart;
                        RayDirection.Normalize();

                        float NormalAlignment = FVector::DotProduct(HitSurfaceNormal, -RayDirection);
                        NormalAlignment *= NormalAlignment;
                        NormalAlignment *= NormalAlignment;
                        NormalAlignment *= NormalAlignment;
                        NormalAlignment *= NormalAlignment;
                        NormalAlignment *= NormalAlignment;    // pow 32
                        // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, FLinearColor::LerpUsingHSV(ColorHit, ColorReflected,
                        // NormalAlignment), 10, .5, dt); NormalAlignment =
                        // (NormalAlignment*(IntensityReflective-IntensityNonReflective) + IntensityNonReflective)/IntensityMax;
                        // LineBatcher->DrawPoint(h.ImpactPoint, InterpolateColor(NormalAlignment), 5, 10, dt);
                        LineBatcher->DrawPoint(
                            h.ImpactPoint,
                            InterpColorFromIntensity(GetIntensityFromDist(
                                NormalAlignment * (IntensityReflective - IntensityNonReflective) + IntensityNonReflective,
                                Distance)),
                            5,
                            DrawPointDepthIntensity,
                            Dt);
                    }
                }
                else
                {
                    // UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("no physics material"));
                    // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorHit, 10, .5, dt);
                    LineBatcher->DrawPoint(h.ImpactPoint,
                                           InterpColorFromIntensity(GetIntensityFromDist(IntensityNonReflective, Distance)),
                                           5,
                                           DrawPointDepthIntensity,
                                           Dt);
                }
            }
            else if (ShowLidarRayMisses)
            {
                // LineBatcher->DrawLine(h.TraceStart, h.TraceEnd, ColorMiss, 10, .25, dt);
                LineBatcher->DrawPoint(h.TraceEnd, ColorMiss, 2.5, DrawPointDepthIntensity, Dt);
            }
        }
    }
}

bool URR3DLidarComponent::Visible(AActor* TargetActor)
{
    TArray<FHitResult> RecordedVizHits;
    RecordedVizHits.Init(FHitResult(ForceInit), GetTotalScan());

    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);
    DVAngle = FOVVertical / static_cast<float>(NChannelsPerScan);

    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("3DLaser_Trace"), true, GetOwner());
    TraceParams.bReturnPhysicalMaterial = true;
    // TraceParams.bIgnoreTouches = true;
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnFaceIndex = true;

    if (bIgnoreSelf)
    {
        TraceParams.AddIgnoredActor(GetOwner());
    }

    FVector lidarPos = GetComponentLocation();
    FRotator lidarRot = GetComponentRotation();

    ParallelFor(
        RecordedVizHits.Num(),
        [this, &TraceParams, &lidarPos, &lidarRot, &RecordedVizHits](int32 Index)
        {
            const int IdxX = Index % NSamplesPerScan;
            const int IdxY = Index / NSamplesPerScan;
            const float HAngle = StartAngle + DHAngle * IdxX;
            const float VAngle = StartVerticalAngle + DVAngle * IdxY;

            FRotator laserRot(VAngle, HAngle, 0);
            FRotator rot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

            FVector startPos = lidarPos + MinRange * UKismetMathLibrary::GetForwardVector(rot);
            FVector endPos = lidarPos + MaxRange * UKismetMathLibrary::GetForwardVector(rot);

            GetWorld()->LineTraceSingleByChannel(RecordedVizHits[Index],
                                                 startPos,
                                                 endPos,
                                                 TraceCollisionChannel,
                                                 TraceParams,
                                                 FCollisionResponseParams::DefaultResponseParam);
        },
        false);

    for (auto& h : RecordedVizHits)
    {
        if (h.GetActor() == TargetActor)
        {
            return true;
        }
    }
    return false;
}

FROSPointCloud2 URR3DLidarComponent::GetROS2Data()
{
    FROSPointCloud2 retValue;

    // time
    retValue.Header.Stamp = URRConversionUtils::FloatToROSStamp(TimeOfLastScan);

    retValue.Header.FrameId = FrameId;

    // reference
    // https://github.com/ToyotaResearchInstitute/velodyne_simulator
    static constexpr uint32_t POINT_STEP = 22;
    retValue.PointStep = POINT_STEP;

    retValue.bIsBigendian = false;

    retValue.Fields.Init(FROSPointField(), 6);
    retValue.Fields[0].Name = "x";
    retValue.Fields[0].Offset = 0;
    retValue.Fields[0].Datatype = FROSPointField::FLOAT32;
    retValue.Fields[0].Count = 1;
    retValue.Fields[1].Name = "y";
    retValue.Fields[1].Offset = 4;
    retValue.Fields[1].Datatype = FROSPointField::FLOAT32;
    retValue.Fields[1].Count = 1;
    retValue.Fields[2].Name = "z";
    retValue.Fields[2].Offset = 8;
    retValue.Fields[2].Datatype = FROSPointField::FLOAT32;
    retValue.Fields[2].Count = 1;
    retValue.Fields[3].Name = "intensity";
    retValue.Fields[3].Offset = 12;
    retValue.Fields[3].Datatype = FROSPointField::FLOAT32;
    retValue.Fields[3].Count = 1;
    retValue.Fields[4].Name = "ring";
    retValue.Fields[4].Offset = 16;
    retValue.Fields[4].Datatype = FROSPointField::UINT16;
    retValue.Fields[4].Count = 1;
    retValue.Fields[5].Name = "time";
    retValue.Fields[5].Offset = 18;
    retValue.Fields[5].Datatype = FROSPointField::FLOAT32;
    retValue.Fields[5].Count = 1;

    retValue.Data.Init(0, RecordedHits.Num() * POINT_STEP);

    int count = 0;
    for (auto i = 0; i < NChannelsPerScan; i++)
    {
        for (auto j = 0; j < NSamplesPerScan; j++)
        {
            int index = j + i * NSamplesPerScan;
            // float Distance = (MinRange * (RecordedHits.Last(index).Distance > 0) + RecordedHits.Last(index).Distance) * .01f;
            // Distance += BWithNoise * IntensityNoise->Get();
            FVector3f pos = FVector3f::ZeroVector;

            const float IntensityScale = 1.f + BWithNoise * IntensityNoise->Get();
            float Intensity = 0.0;
            if (RecordedHits.Last(index).PhysMaterial != nullptr)
            {
                // retroreflective material
                if (RecordedHits.Last(index).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType1)
                {
                    Intensity = IntensityScale * IntensityReflective;
                }
                // non-reflective material
                else if (RecordedHits.Last(index).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType_Default)
                {
                    Intensity = IntensityScale * IntensityNonReflective;
                }
                // reflective material
                else if (RecordedHits.Last(index).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType2)
                {
                    FVector HitSurfaceNormal = RecordedHits.Last(index).Normal;
                    FVector RayDirection = RecordedHits.Last(index).TraceEnd - RecordedHits.Last(index).TraceStart;
                    RayDirection.Normalize();

                    // the dot product for this should always be between 0 and 1
                    const float UnnormalizedIntensity =
                        FMath::Clamp(IntensityNonReflective + (IntensityReflective - IntensityNonReflective) *
                                                                  FVector::DotProduct(HitSurfaceNormal, -RayDirection),
                                     IntensityNonReflective,
                                     IntensityReflective);
                    if ((UnnormalizedIntensity <= IntensityNonReflective) || (UnnormalizedIntensity <= IntensityReflective))
                    {
                        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Normalized intensity is outof range. Something is wrong."));
                    }
                    Intensity = IntensityScale * UnnormalizedIntensity;
                }
            }
            else
            {
                Intensity = 0;    // std::numeric_limits<float>::quiet_NaN();
                if (!bOrganizedCloud)
                {
                    continue;
                }
            }

            // Convert pose to local coordinate, ROS unit and double -> float
            FVector posInDouble = RecordedHits.Last(index).ImpactPoint;    // + BWithNoise * PositionNoise->Get();
            posInDouble = URRGeneralUtils::GetRelativeTransform(
                              FTransform(GetComponentQuat(), GetComponentLocation(), FVector::OneVector), FTransform(posInDouble))
                              .GetTranslation();
            posInDouble = URRConversionUtils::VectorUEToROS(posInDouble);
            pos = FVector3f(posInDouble);

            float time = 0.f;    //temp
            memcpy(&retValue.Data[count * POINT_STEP], &pos.X, 4);
            memcpy(&retValue.Data[count * POINT_STEP + 4], &pos.Y, 4);
            memcpy(&retValue.Data[count * POINT_STEP + 8], &pos.Z, 4);
            memcpy(&retValue.Data[count * POINT_STEP + 12], &Intensity, 4);
            memcpy(&retValue.Data[count * POINT_STEP + 16], &j, 2);
            memcpy(&retValue.Data[count * POINT_STEP + 18], &time, 4);

            count++;
        }
    }

    if (bOrganizedCloud)
    {
        retValue.Width = NSamplesPerScan;
        retValue.Height = NChannelsPerScan;
        retValue.RowStep = POINT_STEP * NSamplesPerScan;
        retValue.bIsDense = true;
    }
    else
    {
        retValue.Data.SetNum((count + 1) * POINT_STEP, true);
        retValue.Height = 1;
        retValue.Width = count + 1;
        retValue.RowStep = retValue.Data.Num();
        retValue.bIsDense = true;
    }

    return retValue;
}

void URR3DLidarComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2PointCloud2Msg>(InMessage)->SetMsg(GetROS2Data());
}
