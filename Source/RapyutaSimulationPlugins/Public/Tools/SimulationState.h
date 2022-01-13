// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROS2Node.h"

#include "SimulationState.generated.h"

class UROS2GenericSrv;

UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ASimulationState : public AActor
{
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    ASimulationState();

public:
    UFUNCTION(BlueprintCallable)
    void Init(AROS2Node* InROS2Node);

    UFUNCTION(BlueprintCallable)
    void GetEntityStateSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void SetEntityStateSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void AttachSrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void SpawnEntitySrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void DeleteEntitySrv(UROS2GenericSrv* Service);

    UFUNCTION(BlueprintCallable)
    void AddEntity(AActor* Entity);

    void LeftToRight(double& pos_x, double& pos_y, double& pos_z, FQuat& orientation);

    void LeftToRight(FVector& position, FQuat& orientation);

    bool ReferenceFrameToInertiaFrame(const FString& InReferenceFrame,
                                      double& InPositionX,
                                      double& InPositionY,
                                      double& InPositionZ,
                                      FQuat& InOrientation);

    // need node that will handle services - this class will only define and register the service
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    AROS2Node* ROSServiceNode = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, AActor*> Entities;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TMap<FString, TSubclassOf<AActor>> SpawnableEntities;

    // need to keep track of "Entities"? or just use a search?
};
