/**
 * @file RRNetworkPlayerController.h
 * @brief AMR Network Player controller to possess ARRBaseRobot in a client-server application
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */
#pragma once

// UE
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "UnrealClient.h"

// rclUE
#include "ROS2Node.h"

// RapyutaSimulationPlugins
#include "Core/RRActorCommon.h"
#include "Tools/RRROS2ClockPublisher.h"
#include "Tools/RRROS2SimulationStateClient.h"
#include "Tools/SimulationState.h"

#include "RRNetworkPlayerController.generated.h"

/**
 * @brief Network Player controller with its own {ROS2 Node + Clock publisher + ROS2 Sim state client} created for each possessed
 * robot
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRNetworkPlayerController : public APlayerController
{
    GENERATED_BODY()
public:
    ARRNetworkPlayerController();
    static constexpr const TCHAR* CMDLINE_ARG_NET_CLIENT_ROBOT_NAME = TEXT("robotname");

    virtual void Tick(float DeltaSeconds) override;
    void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

    // ROS2
    UPROPERTY(Transient, Replicated)
    AROS2Node* SimStateClientROS2Node = nullptr;

    UPROPERTY(BlueprintReadOnly, Replicated)
    URRROS2ClockPublisher* SimStateClientClockPublisher = nullptr;

    /**
     * @brief Init SimStateClient's ROS2 (Node, ClockPublisher, etc.)
     */
    void InitSimStateClientROS2();

    // SIM STATE & ROS2 STATE CLIENT
    //! Pointer to ServerSimState
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    ASimulationState* ServerSimState = nullptr;
    //! ROS2 Sim state client
    UPROPERTY(Transient, Replicated)
    URRROS2SimulationStateClient* ROS2SimStateClient = nullptr;
    /**
     * @brief Create ROS2SimStateClient without initializing yet
     */
    void CreateROS2SimStateClient(const TSubclassOf<URRROS2SimulationStateClient>& InSimStateClientClass);

    // POSSESSED PAWN
    //! Pawn that has been possessed by this controller
    UPROPERTY(Transient, Replicated)
    APawn* PossessedPawn = nullptr;

    /**
     * @brief Search for the targeted pawn to possess
     */
    APawn* FindPawnToPossess();

    /**
     * @brief Wait to possess the targeted pawn
     */
    UFUNCTION()
    void WaitToPossessPawn();

    /**
     * @brief Possess a pawn on server
     */
    UFUNCTION(Server, Reliable)
    void ServerPossessPawn(APawn* InPawn);

    //! Timer handle used by WaitToPossessPawn()
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle PossessTimerHandle;

    //! Client - Init possessed robot's ROS2Interface & MoveComponent, etc.
    UFUNCTION(Client, Reliable)
    void ClientInitPawn(AActor* InActor);

    // PLAYER
    //! Controller's PlayerName, taken from [robotname] param in Engine build.
    //! In Editor build, it will be automatically set from #PossessedPawn's name upon possession
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"), Replicated)
    FString PlayerName;

    /**
     * @brief Possess a pawn on server
     */
    UFUNCTION(Server, Reliable)
    void ServerSetPlayerName(const FString& InPlayerName);

    // LOCAL CLOCK
    //! Timer handle used by RequestServerTime()
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FTimerHandle ClockRequestTimerHandle;

    //! Local clock time
    UPROPERTY()
    float LocalTime = 0.0f;
    virtual float GetLocalTime()
    {
        return LocalTime;
    }
    /**
     * @brief Request server time update
     */
    void RequestServerTimeUpdate();

    /**
     * @brief TODO
     */
    UFUNCTION(Client, Reliable)
    void ServerSendLocalClockUpdate(float InClientRequestTime, float InServerCurrentTime);

    /**
     * @brief TODO
     */
    UFUNCTION(Server, Reliable)
    void ClientRequestLocalClockUpdate(float InClientRequestTime);

    /**
     * @brief Increase local clock's time by delta seconds
     * @param InDeltaSeconds
     */
    void UpdateLocalClock(float InDeltaSeconds);

protected:
    virtual void BeginPlay() override;
    virtual void ReceivedPlayer() override;
    virtual void AcknowledgePossession(APawn* InPawn) override;
};
