// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Sensors/RR2DLidarComponent.h"

// UE
#include "Async/ParallelFor.h"

// rclUE
#include "rclcUtilities.h"

URR2DLidarComponent::URR2DLidarComponent()
{
    TopicName = TEXT("scan");
    MsgClass = UROS2LaserScanMsg::StaticClass();
}

void URR2DLidarComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
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

void URR2DLidarComponent::Run()
{
    RecordedHits.Init(FHitResult(ForceInit), NSamplesPerScan);

#if TRACE_ASYNC
    TraceHandles.Init(FTraceHandle(), NSamplesPerScan);
#endif

    Super::Run();
}

void URR2DLidarComponent::SensorUpdate()
{
    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);

    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("2DLaser_Trace"), true, GetOwner());
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
            const float HAngle = StartAngle + DHAngle * i;

            FRotator laserRot(0, HAngle, 0);
            FRotator rot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

            FVector startPos = lidarPos + MinRange * UKismetMathLibrary::GetForwardVector(rot);
            FVector endPos = lidarPos + MaxRange * UKismetMathLibrary::GetForwardVector(rot);
            // To be considered: += WithNoise * FVector(GaussianRNGPosition(Gen),GaussianRNGPosition(Gen),GaussianRNGPosition(Gen));

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
            const float HAngle = StartAngle + DHAngle * Index;

            FRotator laserRot(0, HAngle, 0);
            FRotator rot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

            FVector startPos = lidarPos + MinRange * UKismetMathLibrary::GetForwardVector(rot);
            FVector endPos = lidarPos + MaxRange * UKismetMathLibrary::GetForwardVector(rot);
            // + WithNoise *  FVector(GaussianRNGPosition(Gen),GaussianRNGPosition(Gen),GaussianRNGPosition(Gen));

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
    if (LineBatcher != nullptr && bShowLidarRays &&
        IsVisible())    //  && GetParentActor()->GetRootComponent()->IsVisible() ) => compilation error
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

bool URR2DLidarComponent::Visible(AActor* TargetActor)
{
    TArray<FHitResult> RecordedVizHits;
    RecordedVizHits.Init(FHitResult(ForceInit), NSamplesPerScan);

    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);

    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("2DLaser_Trace"), true, GetOwner());
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
            const float HAngle = StartAngle + DHAngle * Index;

            FRotator laserRot(0, HAngle, 0);
            FRotator rot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

            FVector startPos = lidarPos + MinRange * UKismetMathLibrary::GetForwardVector(rot);
            FVector endPos = lidarPos + MaxRange * UKismetMathLibrary::GetForwardVector(rot);
            // To be considered: + WithNoise * FVector(GaussianRNGPosition(Gen),GaussianRNGPosition(Gen),GaussianRNGPosition(Gen));

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
        if (h.GetActor() == TargetActor)
        {
            return true;
        }
    }
    return false;
}

float URR2DLidarComponent::GetMinAngleRadians() const
{
    return FMath::DegreesToRadians(-StartAngle - FOVHorizontal);
}

float URR2DLidarComponent::GetMaxAngleRadians() const
{
    return FMath::DegreesToRadians(-StartAngle);
}

FROSLaserScan URR2DLidarComponent::GetROS2Data()
{
    FROSLaserScan retValue;

    // time
    retValue.Header.Stamp = URRConversionUtils::FloatToROSStamp(TimeOfLastScan);

    retValue.Header.FrameId = FrameId;

    retValue.AngleMin = GetMinAngleRadians();
    retValue.AngleMax = GetMaxAngleRadians();
    retValue.AngleIncrement = FMath::DegreesToRadians(DHAngle);
    retValue.TimeIncrement = Dt / NSamplesPerScan;
    retValue.ScanTime = Dt;
    retValue.RangeMin = MinRange * .01f;
    retValue.RangeMax = MaxRange * .01f;

    retValue.Ranges.Empty();
    retValue.Intensities.Empty();
    // note that angles are reversed compared to rviz
    // ROS is right handed
    // UE4 is left handed
    for (auto i = 0; i < RecordedHits.Num(); i++)
    {
        // convert to [m]
        retValue.Ranges.Add((MinRange * (RecordedHits.Last(i).Distance > 0) + RecordedHits.Last(i).Distance) * .01f);

        const float IntensityScale = 1.f + BWithNoise * GaussianRNGIntensity(Gen);

        UStaticMeshComponent* ComponentHit = Cast<UStaticMeshComponent>(RecordedHits.Last(i).GetComponent());
        if (RecordedHits.Last(i).PhysMaterial != nullptr)
        {
            // retroreflective material
            if (RecordedHits.Last(i).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType1)
            {
                retValue.Intensities.Add(IntensityScale * IntensityReflective);
            }
            // non-reflective material
            else if (RecordedHits.Last(i).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType_Default)
            {
                retValue.Intensities.Add(IntensityScale * IntensityNonReflective);
            }
            // reflective material
            else if (RecordedHits.Last(i).PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType2)
            {
                FVector HitSurfaceNormal = RecordedHits.Last(i).Normal;
                FVector RayDirection = RecordedHits.Last(i).TraceEnd - RecordedHits.Last(i).TraceStart;
                RayDirection.Normalize();

                // the dot product for this should always be between 0 and 1
                const float Intensity =
                    FMath::Clamp(IntensityNonReflective + (IntensityReflective - IntensityNonReflective) *
                                                              FVector::DotProduct(HitSurfaceNormal, -RayDirection),
                                 IntensityNonReflective,
                                 IntensityReflective);
                if ((Intensity <= IntensityNonReflective) || (Intensity <= IntensityReflective))
                {
                    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Intensity is outof range. Something is wrong."));
                }
                retValue.Intensities.Add(IntensityScale * Intensity);
            }
        }
        else
        {
            retValue.Intensities.Add(std::numeric_limits<double>::quiet_NaN());
        }
    }

    return retValue;
}

void URR2DLidarComponent::SetROS2Msg(UROS2GenericMsg* InMessage)
{
    CastChecked<UROS2LaserScanMsg>(InMessage)->SetMsg(GetROS2Data());
}
