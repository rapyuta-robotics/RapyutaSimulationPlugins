/**
 * @file RRUIWidgetComponent.h
 * @brief
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "RRUIWidgetComponent.generated.h"

/**
 * @brief
 * @sa []()
 */
UCLASS(ClassGroup = (Custom), Blueprintable, meta = (BlueprintSpawnableComponent))
class RAPYUTASIMULATIONPLUGINS_API URRUIWidgetComponent : public UWidgetComponent
{
    GENERATED_BODY()

public:
    URRUIWidgetComponent();

    //! Widget class
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Classes)
    TSubclassOf<UUserWidget> UIUserWidgetClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FIntPoint UIWidgetSize = FIntPoint(500.f, 50.f);

    UFUNCTION(BlueprintCallable)
    void Init();

    /**
     * @brief Set robot's tooltip text through #UIWidgetComp's label
     * @param InTooltip
     */
    UFUNCTION(BlueprintCallable)
    void SetTooltipText(const FString& InTooltip);

    /**
     * @brief Toggle robot's tooltip visibility
     * @param bInTooltipVisible
     */
    UFUNCTION(BlueprintCallable)
    void SetTooltipVisible(bool bInTooltipVisible);

    /**
     * @brief Set visibility of #UIUserWidget
     * @param bInWidgetVisible
     */
    UFUNCTION(BlueprintCallable)
    void SetUIWidgetVisible(bool bInWidgetVisible);

    UFUNCTION(BlueprintCallable)
    URRUserWidget* GetRRWidget() const
    {
        return UIUserWidget;
    }

private:
    //! #UUserWidget's widget
    TObjectPtr<URRUserWidget> UIUserWidget = nullptr;
};
