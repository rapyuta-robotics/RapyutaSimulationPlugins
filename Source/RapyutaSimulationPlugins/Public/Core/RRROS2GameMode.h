/**
 * @file RRROS2GameMode.h
 * @brief ROS 2 GameMode which have Clock publisher and ROS 2 services to interact with ROS 2.
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "Engine/EngineTypes.h"
#include "GameFramework/GameMode.h"
#include "Engine/Engine.h"
#include "Tools/RRLimitRTFFixedSizeCustomTimeStep.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2SimulationStateClient.h"
#include "Tools/SimulationState.h"

#include "RRROS2GameMode.generated.h"

class AROS2Node;
class URRROS2ClockPublisher;

DECLARE_MULTICAST_DELEGATE(FRROnROS2Initialized);

/**
 * @brief ROS 2 GameMode which have Clock publisher and ROS 2 services to interact with ROS 2.
 * @sa [AGameMode](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/AGameMode/)
 * @sa [GameMode and GameState](https://docs.unrealengine.com/5.1/en-US/game-mode-and-game-state-in-unreal-engine/)
 */
UCLASS(Config = RapyutaSimSettings)
class RAPYUTASIMULATIONPLUGINS_API ARRROS2GameMode : public AGameMode
{
    GENERATED_BODY()

public:
    ARRROS2GameMode();
    
    /**
     * @brief Destructor - cleanup RTF logging timer
     */
    virtual ~ARRROS2GameMode();

    //! Sim's Main ROS 2 node. This is not used by client-server and #ARRNetworkPlayerController has ROS2Node instead.
    UPROPERTY(BlueprintReadOnly)
    UROS2NodeComponent* MainROS2Node = nullptr;

    //! Sim's Main ROS 2 node name. This is not used by client-server and #ARRNetworkPlayerController has ROS2Node instead.
    UPROPERTY(BlueprintReadWrite)
    FString MainROS2NodeName = TEXT("UEROS2Node");

    //! Publish /clock. This is not used by client-server without editor and #ARRNetworkPlayerController has ClockPublisher instead.
    UPROPERTY(BlueprintReadOnly)
    URRROS2ClockPublisher* ClockPublisher = nullptr;

    //! Provide ROS 2 implementation of sim-wide operations like get/set actor state, spawn/delete actor, attach/detach actor.
    UPROPERTY(BlueprintReadOnly)
    ASimulationState* MainSimState = nullptr;

    //! Provide ROS 2 interface of sim-wide operations implemented by #MainSimState.
    //! @note #ASimulationState and #URRROS2SimulationStateClient are separated to be used in client-server.
    UPROPERTY(BlueprintReadOnly)
    URRROS2SimulationStateClient* MainROS2SimStateClient = nullptr;

    //! Custom type to instantiate #MainROS2SimStateClient, configurable in child classes
    UPROPERTY(BlueprintReadOnly)
    TSubclassOf<URRROS2SimulationStateClient> ROS2SimStateClientClass = URRROS2SimulationStateClient::StaticClass();

    //! Delegate signalling ROS 2 having been initialized with #MainROS2Node, #MainROS2SimStateClient, #ClockPublisher ready
    FRROnROS2Initialized OnROS2Initialized;



    //! Callback function for periodic RTF logging
    UFUNCTION()
    void LogCurrentRTF();
    /**
     * @brief Set timestep by FApp::SetFixedDeltaTime.
     *
     * @param InFrequency game time update frequency, timestep become 1/frequency.
     * @attention FApp::SetFixedDeltaTime take arg as double but InFreqency is float to be able to access from BP.
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetFixedTimeStep(const float InStepSize);

    /**
     * @brief Get timestep
     *
     * @return fixed timestep.
     */
    UFUNCTION(BlueprintCallable)
    virtual float GetFixedTimeStep() const;

    /**
     * @brief Set the Target RTF. This works if CustomTimeStep class is #URRLimitRTFFixedSizeCustomTimeStep.
     *
     * @param InTargetRTF
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetTargetRTF(const float InTargetRTF);

    /**
     * @brief Get the Target RTF. This works if CustomTimeStep class is #URRLimitRTFFixedSizeCustomTimeStep.
     *
     * @return float
     */
    UFUNCTION(BlueprintCallable)
    virtual float GetTargetRTF() const;

    /**
     * @brief Calculate and get the current Real Time Factor (RTF)
     * RTF = Simulation Time / Real Time
     *
     * @return float Current RTF value
     */
    UFUNCTION(BlueprintCallable)
    virtual float GetCurrentRTF() const;

    /**
     * @brief Reset RTF calculation counters
     */
    UFUNCTION(BlueprintCallable)
    virtual void ResetRTFCalculation();
    
    /**
     * @brief Set RTF calculation window size
     *
     * @param WindowSize Size of the sliding window in seconds
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetRTFWindowSize(float WindowSize);

    /**
     * @brief Enable/disable periodic RTF logging
     *
     * @param bEnable Whether to enable periodic RTF logging
     * @param LogInterval Interval in seconds for RTF logging (default: 5.0s)
     */
    UFUNCTION(BlueprintCallable)
    virtual void SetRTFLogging(bool bEnable, float LogInterval = 5.0f);

    /**
     * @brief Print GameMode's user configs in INI
     */
    virtual void PrintSimConfig() const;

    /**
     * @brief Get corresponding class path of a native entity model from #NativeSpawnableClassPaths
     */
    FString GetSpawnableNativeEntityClassPath(const FString& InEntityModelName) const
    {
        return NativeSpawnableClassPaths.FindRef(InEntityModelName);
    }

protected:
    /**
     * @brief Initialize Game.
     *
     * @param InMapName
     * @param InOptions
     * @param OutErrorMessage
     *
     * @sa [InitGame](https://docs.unrealengine.com/5.1/en-US/API/Runtime/Engine/GameFramework/AGameMode/InitGame/)
     */
    virtual void InitGame(const FString& InMapName, const FString& InOptions, FString& OutErrorMessage) override;

    /**
     * @brief Create #MainSimState and call #InitROS2
     *
     */
    virtual void InitSim();

    /**
     * @brief Set timestep with startplay and call #InitSim
     */
    virtual void StartPlay() override;

    //! Blueprint class names (also used as their entity model names) to be registered as spawnable entity types
    //! Eg: {"BP_TurtlebotBurger", "BP_TurtlebotBurgerVehicle"}
    UPROPERTY(config)
    TArray<FString> BPSpawnableClassNames;

    //! Asset paths of classes to be registered as spawnable entity types
    //! Eg: {{"TurtlebotBurger", "/Script/RapyutaSimulationPlugins.TurtlebotBurger"]}
    UPROPERTY(config)
    TMap<FString /*Entity model name*/, FString /*Class path*/> NativeSpawnableClassPaths;

    //! RTF calculation window size in seconds
    UPROPERTY(BlueprintReadWrite, Category = "RTF")
    float RTFWindowSize = 10.0f;
    
    //! Time when RTF window calculation started (in seconds since application start)
    mutable double RTFWindowStartTime = 0.0;
    
    //! Simulation time when RTF window calculation started
    mutable double RTFWindowStartSimTime = 0.0;
    
    //! Last calculated RTF value (maintained until new window completes)
    mutable float LastCalculatedRTF = 0.0f;
    
    //! Whether RTF calculation has been initialized
    mutable bool bRTFCalculationInitialized = false;

    //! Whether periodic RTF logging is enabled
    UPROPERTY(BlueprintReadWrite, Category = "RTF")
    bool bRTFLoggingEnabled = false;

    //! Interval for RTF logging in seconds
    UPROPERTY(BlueprintReadWrite, Category = "RTF")
    float RTFLoggingInterval = 5.0f;

    //! Timer handle for RTF logging
    FTimerHandle RTFLoggingTimerHandle;

private:
    /**
     * @brief Create and initialize #MainROS2Node, #ClockPublisher and #MainSimState.
     *
     */
    void InitROS2();
};
