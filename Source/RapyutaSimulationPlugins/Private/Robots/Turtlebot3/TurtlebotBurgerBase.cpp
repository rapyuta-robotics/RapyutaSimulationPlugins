// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Robots/Turtlebot3/TurtlebotBurgerBase.h"

DEFINE_LOG_CATEGORY(LogTurtlebotBurger);

ATurtlebotBurgerBase::ATurtlebotBurgerBase(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    ROS2InterfaceClass = URRTurtlebotROS2Interface::StaticClass();
    PrimaryActorTick.bCanEverTick = true;
    bBodyComponentsCreated = false;
    SetupBody();
    SetupConstraintsAndPhysics();
    UE_LOG_WITH_INFO_SHORT(
        LogRapyutaCore, Warning, TEXT("%d, %d"), Base_LidarSensor == nullptr, !Base_LidarSensor->IsAttachedTo(LidarSensor));
}

bool ATurtlebotBurgerBase::SetupBody()
{
    if (bBodyComponentsCreated)
    {
        return false;
    }

    Base = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base"));
    SetBaseMeshComp(Base, true, false);

    LidarSensor = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LidarSensor"));
    LidarComponent = CreateDefaultSubobject<URR2DLidarComponent>(TEXT("LidarComp"));
    WheelLeft = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelLeft"));
    WheelRight = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelRight"));
    CasterBack = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CasterBack"));

    // Since attaching to PhysicsConstraintComponent does not work, set each link pose here.
    LidarSensor->SetupAttachment(Base);
    LidarSensor->SetRelativeLocation(FVector(0, 0, 17.2));
    LidarComponent->SetupAttachment(LidarSensor);
    CasterBack->SetupAttachment(Base);
    CasterBack->SetRelativeLocation(FVector(-4.9, 0, -0.5));

    bBodyComponentsCreated = true;

    // Constraints
    Base_LidarSensor = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_LidarSensor"));
    Base_LidarSensor->SetupAttachment(Base);

    Base_CasterBack = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_CasterBack"));
    Base_CasterBack->SetupAttachment(Base);

    return true;
}

void ATurtlebotBurgerBase::PostInitializeComponents()
{
    Super::PostInitializeComponents();
    SetupWheelDrives();
}

void ATurtlebotBurgerBase::SetupWheelDrives()
{
    UE_LOG_WITH_INFO_SHORT(LogRapyutaCore, Error, TEXT("This method should be implemented in child class."));
}

bool ATurtlebotBurgerBase::SetupConstraintsAndPhysics()
{
    if (bBodyComponentsCreated)
    {
        Base->SetSimulatePhysics(true);
        LidarSensor->SetSimulatePhysics(true);
        WheelLeft->SetSimulatePhysics(true);
        WheelRight->SetSimulatePhysics(true);
        CasterBack->SetSimulatePhysics(true);

        // LidarSensor->SetupAttachment(Base_LidarSensor); // Attach to PhysicsConstraintComponent does not work.
        Base_LidarSensor->ComponentName1.ComponentName = TEXT("Base");
        Base_LidarSensor->ComponentName2.ComponentName = TEXT("LidarSensor");
        Base_LidarSensor->SetRelativeLocation(FVector(0, 0, 17.2));
        Base_LidarSensor->SetDisableCollision(true);
        Base_LidarSensor->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
        Base_LidarSensor->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
        Base_LidarSensor->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0);
        Base_LidarSensor->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_LidarSensor->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_LidarSensor->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

        // CasterBack->SetupAttachment(Base_CasterBack); // Attach to PhysicsConstraintComponent does not work.
        Base_CasterBack->ComponentName1.ComponentName = TEXT("Base");
        Base_CasterBack->ComponentName2.ComponentName = TEXT("CasterBack");
        Base_CasterBack->SetRelativeLocation(FVector(-4.9, 0, -0.5));
        Base_CasterBack->SetDisableCollision(true);
        Base_CasterBack->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_CasterBack->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
        Base_CasterBack->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

        return true;
    }
    else
    {
        UE_LOG_WITH_INFO(LogTurtlebotBurger, Error, TEXT("Turtlebot not initialized - can't setup constraints!"));
        return false;
    }
}
