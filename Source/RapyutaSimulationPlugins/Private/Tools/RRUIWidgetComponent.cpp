// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

#include "Tools/RRUIWidgetComponent.h"
#include "UI/RRUserWidget.h"

URRUIWidgetComponent::URRUIWidgetComponent()
{
    UIUserWidgetClass = URRUserWidget::StaticClass();
}

void URRUIWidgetComponent::Init()
{
    SetWorldScale3D(FVector::ZeroVector);

    // 1- Set [WidgetClass] as [URRUserWidget]
    SetWidgetClass(UIUserWidgetClass);

    // 1.1 - Init [UIWidgetComp]
    InitWidget();
    TObjectPtr<UUserWidget> widget = GetWidget();
    if (widget == nullptr)
    {
        return;
    }
    SetDrawSize(UIWidgetSize);
    SetPivot(FVector2D::ZeroVector);
    SetCanEverAffectNavigation(false);
    SetTwoSided(true);
    // NOTE: Using Screen widget space, [UIWidgetComp] will be always facing user view, thus no need to manually orientate it per Tick
    SetWidgetSpace(EWidgetSpace::Screen);

    // 1.2 - Save [UIWidgetComp]'s widget into [UIUserWidget]
    UIUserWidget = Cast<URRUserWidget>(GetWidget());
    if (UIUserWidget)
    {
        UIUserWidget->OwnerWidgetComponent = this;
        UIUserWidget->SetLabelText(GetOwner()->GetName());
    }
}

void URRUIWidgetComponent::SetTooltipText(const FString& InTooltip)
{
    if (UIUserWidget)
    {
        UIUserWidget->SetLabelText(InTooltip);
    }
}

void URRUIWidgetComponent::SetTooltipVisible(bool bInTooltipVisible)
{
    if (UIUserWidget && UIUserWidget->TextBlock)
    {
        UIUserWidget->TextBlock->SetVisibility(bInTooltipVisible ? ESlateVisibility::Visible : ESlateVisibility::Collapsed);
    }
}

void URRUIWidgetComponent::SetUIWidgetVisible(bool bInWidgetVisible)
{
    TObjectPtr<UUserWidget> widget = GetWidget();
    if (widget)
    {
        widget->SetVisibility(bInWidgetVisible ? ESlateVisibility::Visible : ESlateVisibility::Collapsed);
    }
}
