/**
 * @file RRCharacter.h
 * @brief Base Character class, with navigation support
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once

// UE
#include "AITypes.h"
#include "GameFramework/Character.h"

#include "RRCharacter.generated.h"
/**
 * @brief Base Character class, with navigation support.
 * - AIControllerClass: #URRCrowdAIController
 * - Plugged-in #UNavigationInvokerComponent
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API ARRCharacter : public ACharacter
{
    GENERATED_BODY()
public:
    /**
     * @brief Construct a new ARRCharacter object
     *
     */
    ARRCharacter();

    /**
     * @brief Construct a new ARRCharacter object
     *
     * @param ObjectInitializer
     */
    ARRCharacter(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

    /**
     * @brief Initialize character default components
     *
     */
    void SetupDefaultCharacter();

protected:
    /**
     * @brief Post Initialization process of character. Initialize #NavInvokerComp by calling #InitNavInvokerComponent.
     * @sa[ActorLifecycle](https://docs.unrealengine.com/4.27/en-US/ProgrammingAndScripting/ProgrammingWithCPP/UnrealArchitecture/Actors/ActorLifecycle/)
     * @sa[PostInitializeComponents](https://docs.unrealengine.com/4.27/en-US/API/Runtime/Engine/GameFramework/AActor/PostInitializeComponents/)
     */
    virtual void PostInitializeComponents() override;

    /**
     * @brief Initially configure Character movement comp
     */
    void InitCharacterMovementComp();
};
