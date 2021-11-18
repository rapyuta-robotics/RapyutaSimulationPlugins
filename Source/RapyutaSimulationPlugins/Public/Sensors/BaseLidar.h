// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

// std
#include <random>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "ROS2Node.h"
#include "ROS2Publisher.h"

#include "BaseLidar.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogROS2Sensor, Log, All);

#define TRACE_ASYNC 1

UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API ABaseLidar : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ABaseLidar();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

    UFUNCTION(BlueprintCallable)
    virtual void Run();

    UFUNCTION(BlueprintCallable)
    virtual void Scan();

    UFUNCTION(BlueprintCallable)
    virtual void ScanMultiframe();

    UFUNCTION()
    virtual void DrawLidar();

    UFUNCTION()
    virtual void LidarMessageUpdate(UROS2GenericMsg* TopicMessage);

    UFUNCTION(BlueprintCallable)
    virtual bool Visible(AActor* TargetActor);

    UFUNCTION(BlueprintCallable)
    virtual void InitLidar(AROS2Node* Node, const FString& TopicName);

    UFUNCTION(BlueprintCallable)
    virtual void InitToNode(AROS2Node* Node);

    // adding the rest of the necessary information might be tedious
    // eventually split into multiple getters
    UFUNCTION(BlueprintCallable)
    void GetData(TArray<FHitResult>& OutHits, float& OutTime);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2Publisher* LidarPublisher;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString FrameId = TEXT("base_scan");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NSamplesPerScan = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 ScanFrequency = 0;

    // [degrees]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartAngle = 0.f;

    // scan goes from StartAngle to StartAngle+FOVHorizontal
    // [degrees]
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float FOVHorizontal = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MinRange = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxRange = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FLinearColor ColorMiss = FColor(255, 127, 0, 255);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FLinearColor ColorMin = FColor(255, 0, 0, 255);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FLinearColor ColorMid = FColor(255, 0, 0, 255);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    FColor ColorMax = FColor(255, 255, 255, 255);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float TimeOfLastScan = 0.f;

    // [degrees]
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float DHAngle = 0.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TArray<FHitResult> RecordedHits;

#if TRACE_ASYNC
    TArray<FTraceHandle> TraceHandles;
#endif

    UPROPERTY()
    FTimerHandle TimerHandle;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool ShowLidarRays = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool ShowLidarRayMisses = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityNonReflective = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityReflective = 6000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityMin = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Intensity")
    float IntensityMax = 10000.f;

    FLinearColor GetColorFromIntensity(const float Intensity, const float alpha=1);

    

    // this variable indicates the number of frames used to perform a full scan
    UPROPERTY(VisibleAnywhere)
    int32 NSteps = 1;

    UPROPERTY(VisibleAnywhere)
    int32 NSamplesPerStep = 0;

    UPROPERTY(VisibleAnywhere)
    int32 CurrentBatch = 0;

    UPROPERTY(EditAnywhere)
    bool SingleFrameScan = true;

protected:
    float dt = 0.f;
    bool IsInitialized = false;

    // C++11 RNG for noise
    std::random_device Rng;
    std::mt19937 Gen;
    std::normal_distribution<> GaussianRNGPosition;
    std::normal_distribution<> GaussianRNGIntensity;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float PositionalNoiseMean = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float PositionalNoiseVariance = 1.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float IntensityNoiseMean = 0.f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    float IntensityNoiseVariance = .1f;

    UPROPERTY(EditAnywhere, Category = "Noise")
    bool WithNoise = true;

    FLinearColor InterpolateColor(float x);

    float IntensityFromDist(float BaseIntensity, float Distance);
};
