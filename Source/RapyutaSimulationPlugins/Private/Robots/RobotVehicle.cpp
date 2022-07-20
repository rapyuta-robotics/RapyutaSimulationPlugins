// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Robots/RobotVehicle.h"

#include "Net/UnrealNetwork.h"

// rclUE
#include "Msgs/ROS2TFMsg.h"
#include "ROS2Node.h"

ARobotVehicle::ARobotVehicle()
{
    SetupDefaultRootSkeletal();
    bReplicates = true;
}

void ARobotVehicle::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    UBlueprintGeneratedClass * bpClass = Cast<UBlueprintGeneratedClass>(this->GetClass());
    if (bpClass != nullptr)
    {
        bpClass->GetLifetimeBlueprintReplicationList(OutLifetimeProps);
    }
    DOREPLIFETIME( ARobotVehicle, RobotUniqueName );
    DOREPLIFETIME( ARobotVehicle, Map );
    DOREPLIFETIME( ARobotVehicle, SkeletalMeshComp );
    DOREPLIFETIME( ARobotVehicle, RobotVehicleMoveComponent );
    DOREPLIFETIME( ARobotVehicle, VehicleMoveComponentClass );

}

ARobotVehicle::ARobotVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefaultRootSkeletal();
}

void ARobotVehicle::SetupDefaultRootSkeletal()
{
    // Generally, for sake of dynamic robot type import/creation, child components would be then created on the fly!
    // Besides, a default subobject, upon content changes, also makes the owning actor become vulnerable since one in child BP actor
    // classes will automatically get invalidated.

    // [SkeletalMeshComp], due to the support for being configured in certain [ARobotVehicle]'s BP classes,
    // needs to be created in ctor.
    // Reference: AWheeledVehiclePawn
    SkeletalMeshComp = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("SkeletalMeshComp"));
    SkeletalMeshComp->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPose;
    SkeletalMeshComp->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    AddOwnedComponent(SkeletalMeshComp);
    RootComponent = SkeletalMeshComp;

    AIControllerClass = ARRRobotVehicleROSController::StaticClass();
    AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;

}

bool ARobotVehicle::InitSensors(AROS2Node* InROS2Node)
{
    if (false == IsValid(InROS2Node))
    {
        return false;
    }

    // (NOTE) Use [ForEachComponent] would cause a fatal log on
    // [Container has changed during ranged-for iteration!]
    TInlineComponentArray<URRROS2BaseSensorComponent*> sensorComponents(this);
    for (auto& sensorComp : sensorComponents)
    {
        sensorComp->InitalizeWithROS2(InROS2Node);
    }

    return true;
}

bool ARobotVehicle::InitMoveComponent()
{
    if (VehicleMoveComponentClass)
    {
        // (NOTE) Being created in [OnConstruction], PIE will cause this to be reset anyway, thus requires recreation
        RobotVehicleMoveComponent = NewObject<URobotVehicleMovementComponent>(
            this, VehicleMoveComponentClass, *FString::Printf(TEXT("%s_MoveComp"), *GetName()));
        RobotVehicleMoveComponent->RegisterComponent();

        // Configure custom properties (frameids, etc.)
        ConfigureVehicleMoveComponent();

        // Init
        RobotVehicleMoveComponent->Initialize();

        // (NOTE) With [bAutoRegisterUpdatedComponent] as true by default, UpdatedComponent component will be automatically set
        // to the owner actor's root
        return true;
    }
    else
    {
        // [OnConstruction] could run in various Editor BP actions, thus could not do Fatal log here
        UE_LOG(LogRapyutaCore, Warning, TEXT("[%s] [VehicleMoveComponentClass] has not been configured!"), *GetName());
        return false;
    }
}

void ARobotVehicle::SetLinearVel(const FVector& InLinearVelocity)
{
    SetServerLinearVel(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), this->GetActorLocation(), InLinearVelocity);

    SetClientLinearVel(InLinearVelocity);
}

void ARobotVehicle::SetAngularVel(const FVector& InAngularVelocity)
{
    SetServerAngularVel(GetWorld()->GetGameState()->GetServerWorldTimeSeconds(), this->GetActorRotation(), InAngularVelocity);
    SetClientAngularVel(InAngularVelocity);
}

void ARobotVehicle::SetServerLinearVel_Implementation(float TimeStamp, const FVector& InPosition, const FVector& InLinearVelocity)
{
    float ServerCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    this->SetActorLocation(InPosition + InLinearVelocity*(ServerCurrentTime-TimeStamp), false, 0, ETeleportType::None);

    RobotVehicleMoveComponent->Velocity = InLinearVelocity;

}

void ARobotVehicle::SetServerAngularVel_Implementation(float TimeStamp,const FRotator& InRotation, const FVector& InAngularVelocity)
{
    float ServerCurrentTime = UGameplayStatics::GetRealTimeSeconds(GetWorld());
    this->SetActorRotation(InRotation + InAngularVelocity.Rotation()*(ServerCurrentTime-TimeStamp), ETeleportType::None);

    RobotVehicleMoveComponent->AngularVelocity = InAngularVelocity;

}

void ARobotVehicle::SetClientLinearVel_Implementation(const FVector& InLinearVelocity)
{
    APlayerController* Player = UGameplayStatics::GetPlayerController(GetWorld(), 0);
    UE_LOG(LogTemp, Warning, TEXT("PLAYER [%s]"), *Player->PlayerState->GetPlayerName());
    RobotVehicleMoveComponent->Velocity = InLinearVelocity;
}

void ARobotVehicle::SetClientAngularVel_Implementation(const FVector& InAngularVelocity)
}
