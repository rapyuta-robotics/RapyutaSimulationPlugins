// Fill out your copyright notice in the Description page of Project Settings.

#include "Sensors/SensorLidar3D.h"

ASensorLidar3D::ASensorLidar3D() : ABaseLidar()
{
    LidarPublisher = CreateDefaultSubobject<UROS2Publisher>(TEXT("LidarPublisher"));
    LidarPublisher->TopicName = TEXT("scan");
    LidarPublisher->PublicationFrequencyHz = ScanFrequency;
    LidarPublisher->MsgClass = UROS2PointCloud2Msg::StaticClass();
}

void ASensorLidar3D::LidarMessageUpdate(UROS2GenericMsg* TopicMessage)
{
    UROS2PointCloud2Msg* ScanMessage = Cast<UROS2PointCloud2Msg>(TopicMessage);
    ScanMessage->SetMsg(GetROS2Data());
}

void ASensorLidar3D::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
#if TRACE_ASYNC
    UWorld* world = GetWorld();
    for (auto i = 0; i < NSamplesPerScan * NChannelsPerScan; ++i)
    {
        if (TraceHandles[i]._Data.FrameNumber != 0)
        {
            FTraceDatum Output;

            if (world->QueryTraceData(TraceHandles[i], Output))
            {
                if (Output.OutHits.Num() > 0)
                {
                    check(Output.OutHits.Num() > 0);
                    check(i < RecordedHits.Num());
                    check(i < TraceHandles.Num());
                    TraceHandles[i]._Data.FrameNumber = 0;
                    // We should only be tracing the first hit anyhow
                    RecordedHits[i] = Output.OutHits[0];
                }
                else
                {
                    TraceHandles[i]._Data.FrameNumber = 0;
                    RecordedHits[i] = FHitResult();
                    RecordedHits[i].TraceStart = Output.Start;
                    RecordedHits[i].TraceEnd = Output.End;
                }
            }
        }
    }
#endif
}

void ASensorLidar3D::Run()
{
    RecordedHits.Empty();
    RecordedHits.Init(FHitResult(ForceInit), NSamplesPerScan * NChannelsPerScan);

#if TRACE_ASYNC
    TraceHandles.Empty();
    TraceHandles.Init(FTraceHandle{}, NSamplesPerScan * NChannelsPerScan);
#endif

    if (SingleFrameScan)
    {
        GetWorld()->GetGameInstance()->GetTimerManager().SetTimer(
            TimerHandle, this, &ASensorLidar3D::Scan, 1.f / static_cast<float>(ScanFrequency), true);
    }
    else
    {
        // assume fixed framerate of 100FPS - need to find a way to get this information not using DeltaTime
        NSteps = 1.f / (.01f * ScanFrequency);
        NSamplesPerStep = NSamplesPerScan * NChannelsPerScan / NSteps;
        GetWorld()->GetGameInstance()->GetTimerManager().SetTimer(
            TimerHandle, this, &ASensorLidar3D::ScanMultiframe, .01f, true);
    }

    IsInitialized = true;
}

void ASensorLidar3D::ScanMultiframe()
{
    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);
    DVAngle = FOVVertical / static_cast<float>(NChannelsPerScan);

    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
    TraceParams.bReturnPhysicalMaterial = true;

    // TraceParams.bIgnoreTouches = true;
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnFaceIndex = true;

    FVector lidarPos = GetActorLocation();
    FRotator lidarRot = GetActorRotation();

    ParallelFor(
        NSamplesPerStep,
        [this, &TraceParams, &lidarPos, &lidarRot](int32 Index)
        {
            const int32 GlobalIndex = Index + CurrentBatch * NSamplesPerStep;
            const int IdxX = GlobalIndex % NSamplesPerScan;
            const int IdxY = GlobalIndex / NSamplesPerScan;
            const float HAngle = StartAngle + DHAngle * IdxX;
            const float VAngle = StartVerticalAngle + DVAngle * IdxY;

            FRotator laserRot(VAngle, HAngle, 0);
            FRotator rot = UKismetMathLibrary::ComposeRotators(laserRot, lidarRot);

            FVector startPos = lidarPos + MinRange * UKismetMathLibrary::GetForwardVector(rot);
            FVector endPos = lidarPos + MaxRange * UKismetMathLibrary::GetForwardVector(rot);
            // + WithNoise *  FVector(GaussianRNGPosition(Gen),GaussianRNGPosition(Gen),GaussianRNGPosition(Gen));

            GetWorld()->LineTraceSingleByChannel(
                RecordedHits[Index], startPos, endPos, ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);
        },
        false);

    if (CurrentBatch == 0)
    {
        TimeOfLastScan = UGameplayStatics::GetTimeSeconds(GetWorld());
        dt = 1.f / static_cast<float>(ScanFrequency);
    }

    if (ShowLidarRays)
    {
        DrawLidar();
    }

    CurrentBatch = (CurrentBatch + 1) % NSteps;
}

void ASensorLidar3D::Scan()
{
    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);
    DVAngle = FOVVertical / static_cast<float>(NChannelsPerScan);

    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
    TraceParams.bReturnPhysicalMaterial = true;

    // TraceParams.bIgnoreTouches = true;
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnFaceIndex = true;

    FVector lidarPos = GetActorLocation();
    FRotator lidarRot = GetActorRotation();

#if TRACE_ASYNC
    // This is cheesy, but basically if the first trace is in flight we assume they're all waiting and don't do another trace.
    // This is not good if done on other threads and only works because both timers and actor ticks happen on the game thread.
    if (TraceHandles[0]._Data.FrameNumber == 0)
    {
        UWorld* world = GetWorld();
        for (auto i = 0; i < NSamplesPerScan * NChannelsPerScan; ++i)
        {
            const int IdxX = i % NSamplesPerScan;
            const int IdxY = i / NSamplesPerScan;
            const float HAngle = StartAngle + DHAngle * IdxX;
            const float VAngle = StartVerticalAngle + DVAngle * IdxY;

            FRotator laserRot(VAngle, HAngle, 0);
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
        NSamplesPerScan * NChannelsPerScan,
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
            // + WithNoise *  FVector(GaussianRNGPosition(Gen),GaussianRNGPosition(Gen),GaussianRNGPosition(Gen));

            GetWorld()->LineTraceSingleByChannel(
                RecordedHits[Index], startPos, endPos, ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);
        },
        false);
#endif

    if (WithNoise)
    {
        // this approach to noise is different from the above:
        // noise on the linetrace input means that the further the hit, the larger the error, while here the error is independent
        // from distance
        ParallelFor(
            NSamplesPerScan * NChannelsPerScan,
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
    dt = 1.f / static_cast<float>(ScanFrequency);

    // need to store on a structure associating hits with time?
    // GetROS2Data needs to get all data since the last Get? or the last within the last time interval?

    if (ShowLidarRays)
    {
        DrawLidar();
    }
}

void ASensorLidar3D::DrawLidar()
{
    ULineBatchComponent* const LineBatcher = GetWorld()->PersistentLineBatcher;
    if (LineBatcher != nullptr)
    {
        // this can be maybe parallelized? a first trial with ParallelFor crashes - need to verify that LineBatchComponent is thread-safe
        for (int i=0; i<RecordedHits.Num(); i++)
        {
            auto& h = RecordedHits[i];
            if (h.Actor != nullptr)
            {
                float Distance = (MinRange * (h.Distance > 0) + h.Distance) * .01f;
                
                // coloring this way does not seem to work - looks as if CurrentBatch is constant over time
                float alpha = 1;//(NSteps > 1) ? static_cast<float>((i/NSamplesPerStep + NSteps-1 - CurrentBatch) % NSteps) / static_cast<float>(NSteps) : 1;

                if (h.PhysMaterial != nullptr)
                {

                    // retroreflective material
                    if (h.PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType1)
                    {
                        // UE_LOG(LogTemp, Warning, TEXT("retroreflective surface type hit"));
                        // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorReflected, 10, .5, dt);
                        LineBatcher->DrawPoint(
                            h.ImpactPoint, GetColorFromIntensity(IntensityFromDist(IntensityReflective, Distance), alpha), 5, 10, dt);
                    }
                    // non reflective material
                    else if (h.PhysMaterial->SurfaceType == EPhysicalSurface::SurfaceType_Default)
                    {
                        // UE_LOG(LogTemp, Warning, TEXT("default surface type hit"));
                        // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorHit, 10, .5, dt);
                        LineBatcher->DrawPoint(
                            h.ImpactPoint, GetColorFromIntensity(IntensityFromDist(IntensityNonReflective, Distance), alpha), 5, 10, dt);
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
                            GetColorFromIntensity(IntensityFromDist(
                                NormalAlignment * (IntensityReflective - IntensityNonReflective) + IntensityNonReflective,
                                Distance), alpha),
                            5,
                            10,
                            dt);
                    }
                }
                else
                {
                    // UE_LOG(LogTemp, Warning, TEXT("no physics material"));
                    // LineBatcher->DrawLine(h.TraceStart, h.ImpactPoint, ColorHit, 10, .5, dt);
                    LineBatcher->DrawPoint(
                        h.ImpactPoint, GetColorFromIntensity(IntensityFromDist(IntensityNonReflective, Distance), alpha), 5, 10, dt);
                }
            }
            else if (ShowLidarRayMisses)
            {
                // LineBatcher->DrawLine(h.TraceStart, h.TraceEnd, ColorMiss, 10, .25, dt);
                LineBatcher->DrawPoint(h.TraceEnd, ColorMiss, 2.5, 10, dt);
            }
        }
    }
}

bool ASensorLidar3D::Visible(AActor* TargetActor)
{
    TArray<FHitResult> RecordedVizHits;
    RecordedVizHits.Init(FHitResult(ForceInit), NSamplesPerScan * NChannelsPerScan);

    DHAngle = FOVHorizontal / static_cast<float>(NSamplesPerScan);
    DVAngle = FOVVertical / static_cast<float>(NChannelsPerScan);

    // complex collisions: true
    FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
    TraceParams.bReturnPhysicalMaterial = true;
    // TraceParams.bIgnoreTouches = true;
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnFaceIndex = true;

    FVector lidarPos = GetActorLocation();
    FRotator lidarRot = GetActorRotation();

    ParallelFor(
        NSamplesPerScan * NChannelsPerScan,
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

void ASensorLidar3D::InitLidar(AROS2Node* Node, const FString& TopicName)
{
    Super::InitLidar(Node, TopicName);

    Run();
}

void ASensorLidar3D::InitToNode(AROS2Node* Node)
{
    if (IsValid(Node))
    {
        check(IsValid(LidarPublisher));

        LidarPublisher->UpdateDelegate.BindDynamic(this, &ASensorLidar3D::LidarMessageUpdate);
        Node->AddPublisher(LidarPublisher);
        LidarPublisher->Init(UROS2QoS::SensorData);
    }
}

FROSPointCloud2 ASensorLidar3D::GetROS2Data()
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
        const float IntensityScale = 1.f + WithNoise * GaussianRNGIntensity(Gen);
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
            Intensity = 0;//std::numeric_limits<float>::quiet_NaN();
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