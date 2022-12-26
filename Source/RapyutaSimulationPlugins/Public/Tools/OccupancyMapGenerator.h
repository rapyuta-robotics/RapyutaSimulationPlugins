/**
 * @file OccupancyMapGenerator.h
 * @brief Actor to Generate 2D occupancy map for navigation/localization with LineTraceSingleByChannel.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "CoreMinimal.h"
#include "Engine/StaticMeshActor.h"
#include "GameFramework/Actor.h"

#include "OccupancyMapGenerator.generated.h"

/**
 * @brief Actor to Generate 2D occupancy map for navigation/localization with LineTraceSingleByChannel.
 * Generate 2D occupancy map with given parameter and save to file with beginplay.
 * How to use: Place this actor to the level, set parameters(select #Map and max vertical height), and play simulation, then map file will be saved.
 * @sa [LineTraceSingleByChannel](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/Engine/UWorld/LineTraceSingleByChannel/)
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API AOccupancyMapGenerator : public AActor
{
    GENERATED_BODY()

public:
    /**
	 * @brief Construct a new AOccupancyMapGenerator object
	 *
	 */
    AOccupancyMapGenerator();

protected:
    /**
	 * @brief Generate occupancy map with given paramters and save to files.
	 * Called when the game starts or when spawned.
	 */
    virtual void BeginPlay() override;

public:
    //! Generate map to cover bounding box of this actor. Please select actor such as ground plane.
    UPROPERTY(EditAnywhere)
    AStaticMeshActor* Map;

    //! [m/pixel]
    UPROPERTY(EditAnywhere)
    float GridRes = 0.05;

    //! [m]
    UPROPERTY(EditAnywhere)
    float MaxVerticalHeight = 10;    // [m]

    UPROPERTY(EditAnywhere)
    FString Filename = "ue4_map";

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TArray<uint8> OccupancyGrid;

    UFUNCTION()
    /**
	 * @brief Save .pgm and .yaml files.
	 *
	 * @param width
	 * @param height
	 * @param originx
	 * @param originy
	 * @return true
	 * @return false
	 */
    bool WriteToFile(int width, int height, float originx, float originy);
};
