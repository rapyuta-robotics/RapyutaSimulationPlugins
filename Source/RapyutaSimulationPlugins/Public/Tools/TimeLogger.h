/**
 * @file TimeLogger.h
 * @brief Log Simulation and Real timestamps to files
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"

#include "TimeLogger.generated.h"

/**
 * @brief Log Simulation and Real timestamps to files.
 *
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ATimeLogger : public AActor
{
    GENERATED_BODY()

public:
    /**
	 * @brief Construct a new ATimeLogger object. Sets default values for this actor's properties
	 *
	 */
    ATimeLogger();

protected:
    /**
	 * @brief Called when the game starts or when spawned
	 *
	 */
    virtual void BeginPlay() override;

public:
    // Called every frame
    /**
	 * @brief  Called every frame. Save simulation time and real time to #SimTimeHistory and #RealTimeHistory .
	 * if time elapsed more than #MaxTime, call #DumpData to save data to files.
	 * @param DeltaTime
	 */
    virtual void Tick(float DeltaTime) override;

    /**
	 * @brief Start saving time stamp.
	 *
	 */
    UFUNCTION(BlueprintCallable)
    void StartTimer();

    /**
	 * @brief Dump data to files.
	 *
	 */
    UFUNCTION(BlueprintCallable)
    void DumpData();

    UPROPERTY()
    TArray<FString> RealTimeHistory;

    UPROPERTY()
    TArray<FString> SimTimeHistory;

    UPROPERTY()
    FTimerHandle timerHandle;

    //! Max time to log.
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxTime = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float StartSimTime = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FDateTime StartRealTime;
};
