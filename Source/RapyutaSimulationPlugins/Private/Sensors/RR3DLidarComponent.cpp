// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RR3DLidarComponent.h"

URR3DLidarComponent::URR3DLidarComponent()
{
    SensorPublisherClass = URRROS2PointCloud2Publisher::StaticClass();
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

    RecordedHits.Init(FHitResult(ForceInit), nTotalScan);

#if TRACE_ASYNC
    TraceHandles.Init(FTraceHandle(), nTotalScan);
#endif
    Super::Run();
}

void URR3DLidarComponent::SensorUpdate()
{
    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);
    DVAngle = FOVVertical / static_cast<float>(NChannelsPerScan);

    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("3DLaser_Trace"), true, GetOwner());
    TraceParams.bReturnPhysicalMaterial = true;

    // TraceParams.bIgnoreTouches = true;
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnFaceIndex = true;

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
            // To be considered: += bWithNoise * FVector(GaussianRNGPosition(Gen),GaussianRNGPosition(Gen),GaussianRNGPosition(Gen));

            TraceHandles[i] = world->AsyncLineTraceByChannel(EAsyncTraceType::Single,
                                                             startPos,
                                                             endPos,
                                                             ECC_Visibility,
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
            // + bWithNoise *  FVector(GaussianRNGPosition(Gen),GaussianRNGPosition(Gen),GaussianRNGPosition(Gen));

            GetWorld()->LineTraceSingleByChannel(
                RecordedHits[Index], startPos, endPos, ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);
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
                RecordedHits[Index].ImpactPoint +=
                    FVector(GaussianRNGPosition(Gen), GaussianRNGPosition(Gen), GaussianRNGPosition(Gen));
                RecordedHits[Index].TraceEnd +=
                    FVector(GaussianRNGPosition(Gen), GaussianRNGPosition(Gen), GaussianRNGPosition(Gen));
            },
            false);
    }

    TimeOfLastScan = UGameplayStatics::GetTimeSeconds(GetWorld());
    Dt = 1.f / static_cast<float>(PublicationFrequencyHz);

    // need to store on a structure associating hits with time?
    // GetROS2Data needs to get all data since the last Get? or the last within the last time interval?

    ULineBatchComponent* const LineBatcher = GetWorld()->PersistentLineBatcher;
    if (LineBatcher != nullptr && ShowLidarRays)
    {
        for (auto& h : RecordedHits)
        {
            if (h.Actor != nullptr)
            {
                float Distance = (MinRange * (h.Distance > 0) + h.Distance) * .01f;
                if (h.PhysMaterial != nullptr)
                {
                    // retroreflective material
                    if (h.PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType1)
                    {
                        // UE_LOG(LogTemp, Warning, TEXT("retroreflective surface type hit"));
                        // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorReflected, 10, .5, dt);
                        LineBatcher->DrawPoint(h.ImpactPoint,
                                               InterpColorFromIntensity(GetIntensityFromDist(IntensityReflective, Distance)),
                                               5,
                                               10,
                                               Dt);
                    }
                    // non reflective material
                    else if (h.PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType_Default)
                    {
                        // UE_LOG(LogTemp, Warning, TEXT("default surface type hit"));
                        // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorHit, 10, .5, dt);
                        LineBatcher->DrawPoint(h.ImpactPoint,
                                               InterpColorFromIntensity(GetIntensityFromDist(IntensityNonReflective, Distance)),
                                               5,
                                               10,
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
                            10,
                            Dt);
                    }
                }
                else
                {
                    // UE_LOG(LogTemp, Warning, TEXT("no physics material"));
                    // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorHit, 10, .5, dt);
                    LineBatcher->DrawPoint(
                        h.ImpactPoint, InterpColorFromIntensity(GetIntensityFromDist(IntensityNonReflective, Distance)), 5, 10, Dt);
                }
            }
            else if (ShowLidarRayMisses)
            {
                // LineBatcher->DrawLine(h.TraceStart, h.TraceEnd, ColorMiss, 10, .25, dt);
                LineBatcher->DrawPoint(h.TraceEnd, ColorMiss, 2.5, 10, Dt);
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
                                                 ECC_Visibility,
                                                 TraceParams,
                                                 FCollisionResponseParams::DefaultResponseParam);
        },
        false);

    for (auto& h : RecordedVizHits)
    {
        if (h.Actor == TargetActor)
        {
            return true;
        }
    }
    return false;
}

FROSPointCloud2 URR3DLidarComponent::GetROS2Data()
{
    FROSPointCloud2 retValue;
    retValue.header_stamp_sec = (int32)TimeOfLastScan;
    uint64 ns = (uint64)(TimeOfLastScan * 1e+09f);
    retValue.header_stamp_nanosec = (uint32)(ns - (retValue.header_stamp_sec * 1e+09));

    retValue.header_frame_id = FrameId;

    retValue.height = NChannelsPerScan;
    retValue.width = NSamplesPerScan;

    retValue.fields_name.Add("x");
    retValue.fields_name.Add("y");
    retValue.fields_name.Add("z");
    retValue.fields_name.Add("distance");
    retValue.fields_name.Add("intensity");

    // what's the measure? bytes?
    retValue.fields_offset.Add(0);
    retValue.fields_offset.Add(4);
    retValue.fields_offset.Add(8);
    retValue.fields_offset.Add(12);
    retValue.fields_offset.Add(16);

    // 7: float
    retValue.fields_datatype.Add(7);
    retValue.fields_datatype.Add(7);
    retValue.fields_datatype.Add(7);
    retValue.fields_datatype.Add(7);
    retValue.fields_datatype.Add(7);

    retValue.fields_count.Add(1);
    retValue.fields_count.Add(1);
    retValue.fields_count.Add(1);
    retValue.fields_count.Add(1);
    retValue.fields_count.Add(1);

    retValue.is_bigendian = false;

    retValue.point_step = sizeof(float) * 5;
    retValue.row_step = sizeof(float) * 5 * NSamplesPerScan;

    retValue.data.Init(0, RecordedHits.Num() * sizeof(float) * 5);
    for (auto i = 0; i < RecordedHits.Num(); i++)
    {
        float Distance = (MinRange * (RecordedHits.Last(i).Distance > 0) + RecordedHits.Last(i).Distance) * .01f;
        const float IntensityScale = 1.f + BWithNoise * GaussianRNGIntensity(Gen);
        float Intensity = 0;
        if (RecordedHits.Last(i).PhysMaterial != nullptr)
        {
            // retroreflective material
            if (RecordedHits.Last(i).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType1)
            {
                Intensity = IntensityScale * IntensityReflective;
            }
            // non-reflective material
            else if (RecordedHits.Last(i).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType_Default)
            {
                Intensity = IntensityScale * IntensityNonReflective;
            }
            // reflective material
            else if (RecordedHits.Last(i).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType2)
            {
                FVector HitSurfaceNormal = RecordedHits.Last(i).Normal;
                FVector RayDirection = RecordedHits.Last(i).TraceEnd - RecordedHits.Last(i).TraceStart;
                RayDirection.Normalize();

                // the dot product for this should always be between 0 and 1
                const float UnnormalizedIntensity =
                    FMath::Clamp(IntensityNonReflective + (IntensityReflective - IntensityNonReflective) *
                                                              FVector::DotProduct(HitSurfaceNormal, -RayDirection),
                                 IntensityNonReflective,
                                 IntensityReflective);
                check(UnnormalizedIntensity >= IntensityNonReflective);
                check(UnnormalizedIntensity <= IntensityReflective);
                Intensity = IntensityScale * UnnormalizedIntensity;
            }
        }
        else
        {
            Intensity = 0;    // std::numeric_limits<float>::quiet_NaN();
        }

        FVector Pos = RecordedHits.Last(i).ImpactPoint * .01f;
        memcpy(&retValue.data[i * 4 * 5], &Pos.X, 4);
        memcpy(&retValue.data[i * 4 * 5 + 4], &Pos.Y, 4);
        memcpy(&retValue.data[i * 4 * 5 + 8], &Pos.Z, 4);
        memcpy(&retValue.data[i * 4 * 5 + 12], &Distance, 4);
        memcpy(&retValue.data[i * 4 * 5 + 16], &Intensity, 4);
    }

    retValue.is_dense = true;

    return retValue;
}

void URR3DLidarComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2PointCloud2Msg>(InMessage)->SetMsg(GetROS2Data());
}