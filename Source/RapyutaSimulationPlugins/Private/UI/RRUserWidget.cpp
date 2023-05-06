// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "UI/RRUserWidget.h"

TSharedRef<SWidget> URRUserWidget::RebuildWidget()
{
    // Propably nullptr: Either because of the lowest-level class or root component not being set yet
    if (!Cast<UPanelWidget>(GetRootWidget()) && WidgetTree)
    {
        WidgetTree->RootWidget = WidgetTree->ConstructWidget<UCanvasPanel>();
    }

    if (nullptr == MainPanelWidget)
    {
        MainPanelWidget = Cast<UPanelWidget>(GetRootWidget());
        verify(MainPanelWidget);

        // This will create MainPanelWidget's slots
        SetupContents();
        verify(MainPanelWidget->GetSlots().Num() > 0);

        UCanvasPanelSlot* canvasSlot = Cast<UCanvasPanelSlot>(MainPanelWidget->GetSlots()[0]);
        canvasSlot->SetAutoSize(true);
        canvasSlot->SetAnchors(FAnchors(0.f, 0.f, 1.f, 1.f));
    }

    // The root widget must be available there before calling Super::RebuildWidget(), which calls TakeWidget()
    // and the ctor for proper root widget initialization.
    return Super::RebuildWidget();
}

void URRUserWidget::SetActivated(bool bIsActivated)
{
    if (bIsActivated)
    {
        if (!IsInViewport())
        {
            // Add it to the screen so the NativeConstruct() method in the UUserWidget:: is run.
            AddToPlayerScreen(1);
        }
        SetVisibility(ESlateVisibility::Visible);
        SetKeyboardFocus();
    }
    else
    {
        SetVisibility(ESlateVisibility::Hidden);
    }
}

void URRUserWidget::SetupContents()
{
    TextBlock = AddChildWidget<UTextBlock>(TEXT("TextBlock"));
}

void URRUserWidget::SetLabelText(const FString& InText)
{
    TextBlock->SetText(FText::FromString(InText));
}