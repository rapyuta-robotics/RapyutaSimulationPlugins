// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Robot/RobotVehicle.h"

#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

ARobotVehicle::ARobotVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
}

void ARobotVehicle::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
}

void ARobotVehicle::SetLinearVel(FVector Velocity)
{
    // We're assuming input is in meters, so convert to centimeters.
    MoveComponent->Velocity = Velocity;
}

void ARobotVehicle::SetAngularVel(FVector Velocity)
{
    MoveComponent->AngularVelocity = Velocity;
}

void ARobotVehicle::BeginPlay()
{
    Super::BeginPlay();
    MoveComponent->InitMovementComponent();
    /*
        for (TFieldIterator<FProperty> PropIt(FTestData::StaticStruct()); PropIt; ++PropIt)
        {
            FProperty *Property = *PropIt;

            FString Name = Property->GetName();
            FString Type = Property->GetCPPType();

            UE_LOG(LogTemp, Warning, TEXT("*** Prop: %s (%s)"), *Name, *Type);
        }
    */
}

void ARobotVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
}
