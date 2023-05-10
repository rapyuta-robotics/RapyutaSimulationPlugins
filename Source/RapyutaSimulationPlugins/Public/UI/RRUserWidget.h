/**
 * @file RRUserWidget.h
 * @brief Base user widget
 * @copyright Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
 */

#pragma once

#include "Blueprint/UserWidget.h"
#include "Components/PanelWidget.h"
#include "Components/TextBlock.h"
#include "Components/WidgetComponent.h"
#include "IUMGModule.h"
#include "Slate/SObjectWidget.h"
#include "UMG.h"
#include "UMGStyle.h"

#include "RRUserWidget.generated.h"

/**
 * @brief Base user widget, inheriting from [UUserWidget](https://docs.unrealengine.com/5.1/en-US/API/Runtime/UMG/Blueprint/UUserWidget)
 * Reference: https://answers.unrealengine.com/questions/470481/create-widget-in-pure-c.html
 */
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRUserWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    UPROPERTY(meta = (BindWidgetOptional))
    UPanelWidget* MainPanelWidget = nullptr;

    //! WidgetComponent is a mesh component as surface in the 3D environment on which to render widgets normally rendered to
    //! the screen. It is first rendered to a render target, then that render target is displayed in the world.
    UPROPERTY()
    TObjectPtr<UWidgetComponent> OwnerWidgetComponent = nullptr;

    UPROPERTY(meta = (BindWidgetOptional))
    UTextBlock* TextBlock = nullptr;

public:
    virtual bool HasValidContents()
    {
        return MainPanelWidget && WidgetTree && TextBlock;
    }

    template<typename TChildWidget>
    TChildWidget* AddChildWidget(const TCHAR* InChildWidgetNameAffix)
    {
        TChildWidget* childWidget = WidgetTree->ConstructWidget<TChildWidget>(TChildWidget::StaticClass(), InChildWidgetNameAffix);
        verify(childWidget);
        verify(MainPanelWidget);
        MainPanelWidget->AddChild(childWidget);
        return childWidget;
    }

    UFUNCTION()
    void SetActivated(bool bIsActivated);

    UFUNCTION()
    void SetLabelText(const FString& InText);

protected:
	//! Invoked as the widget is added to the viewport
    virtual TSharedRef<SWidget> RebuildWidget() override;
    virtual bool NativeSupportsKeyboardFocus() const override
    {
        return true;
    }
    virtual void SetupContents();
};