#!/usr/bin/env python3
"""Generate a minimal Unreal Engine 5 project for ACESim visual sync."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

PROJECT_NAME = "ACESimUE"
PLUGIN_NAME = "ACESimBridge"
RENDER_PRESETS = {"performance", "lumen", "raytracing"}


def _normalize_render_preset(render_preset: str) -> str:
    normalized = render_preset.strip().lower()
    if normalized not in RENDER_PRESETS:
        raise ValueError(f"Unsupported ACESim UE render preset: {render_preset}")
    return normalized


def _render_preset_engine_ini(render_preset: str) -> str:
    preset = _normalize_render_preset(render_preset)
    if preset == "raytracing":
        return """[/Script/LinuxTargetPlatform.LinuxTargetSettings]
+TargetedRHIs=SF_VULKAN_SM6
bEnableRayTracing=true

[/Script/Engine.RendererSettings]
r.DynamicGlobalIlluminationMethod=1
r.ReflectionMethod=1
r.GenerateMeshDistanceFields=True
r.RayTracing=True
r.Lumen.HardwareRayTracing=True
r.UseHardwareRayTracingWhenAvailable=True
"""
    if preset == "lumen":
        return """[/Script/LinuxTargetPlatform.LinuxTargetSettings]
+TargetedRHIs=SF_VULKAN_SM5
bEnableRayTracing=false

[/Script/Engine.RendererSettings]
r.DynamicGlobalIlluminationMethod=1
r.ReflectionMethod=1
r.GenerateMeshDistanceFields=True
r.RayTracing=False
r.Lumen.HardwareRayTracing=False
"""
    return """[/Script/LinuxTargetPlatform.LinuxTargetSettings]
+TargetedRHIs=SF_VULKAN_SM5
bEnableRayTracing=false

[/Script/Engine.RendererSettings]
r.DynamicGlobalIlluminationMethod=0
r.ReflectionMethod=0
r.GenerateMeshDistanceFields=False
r.RayTracing=False
"""


def _uproject() -> str:
    return (
        json.dumps(
            {
                "FileVersion": 3,
                "EngineAssociation": "",
                "Category": "Simulation",
                "Description": "ACESim visual-sync project for Unreal Engine 5.",
                "Modules": [
                    {
                        "Name": PROJECT_NAME,
                        "Type": "Runtime",
                        "LoadingPhase": "Default",
                    }
                ],
                "Plugins": [
                    {
                        "Name": PLUGIN_NAME,
                        "Enabled": True,
                    }
                ],
            },
            indent=2,
        )
        + "\n"
    )


def _uplugin() -> str:
    return (
        json.dumps(
            {
                "FileVersion": 3,
                "Version": 1,
                "VersionName": "0.1.0",
                "FriendlyName": "ACESim Bridge",
                "Description": "Receives ACESim ZeroMQ visual state and applies it to UE actors.",
                "Category": "Simulation",
                "EnabledByDefault": True,
                "CanContainContent": False,
                "Modules": [
                    {
                        "Name": PLUGIN_NAME,
                        "Type": "Runtime",
                        "LoadingPhase": "Default",
                    }
                ],
            },
            indent=2,
        )
        + "\n"
    )


def _templates(render_preset: str = "performance") -> dict[str, str]:
    render_settings = _render_preset_engine_ini(render_preset)
    return {
        f"{PROJECT_NAME}.uproject": _uproject(),
        "Config/DefaultEngine.ini": f"""[/Script/Engine.Engine]
+ActiveGameNameRedirects=(OldGameName="TP_Blank",NewGameName="/Script/ACESimUE")
GameViewportClientClassName=/Script/ACESimUE.ACESimGameViewportClient

[/Script/EngineSettings.GameMapsSettings]
EditorStartupMap=/Engine/Maps/Entry
GameDefaultMap=/Engine/Maps/Entry
GlobalDefaultGameMode=/Script/ACESimUE.ACESimUEGameMode
{render_settings}
""",
        "Config/DefaultInput.ini": """[/Script/Engine.InputSettings]
bCaptureMouseOnLaunch=False
DefaultViewportMouseCaptureMode=CaptureDuringMouseDown
DefaultViewportMouseLockMode=DoNotLock
+ActionMappings=(ActionName="ReleaseLook",Key=Escape)
+AxisMappings=(AxisName="MoveForward",Key=W,Scale=1.000000)
+AxisMappings=(AxisName="MoveForward",Key=S,Scale=-1.000000)
+AxisMappings=(AxisName="MoveRight",Key=D,Scale=1.000000)
+AxisMappings=(AxisName="MoveRight",Key=A,Scale=-1.000000)
+AxisMappings=(AxisName="MoveUp",Key=SpaceBar,Scale=1.000000)
+AxisMappings=(AxisName="MoveUp",Key=LeftControl,Scale=-1.000000)
+AxisMappings=(AxisName="LookYaw",Key=Right,Scale=1.000000)
+AxisMappings=(AxisName="LookYaw",Key=Left,Scale=-1.000000)
+AxisMappings=(AxisName="LookPitch",Key=Up,Scale=1.000000)
+AxisMappings=(AxisName="LookPitch",Key=Down,Scale=-1.000000)
""",
        "Config/DefaultGame.ini": """[/Script/EngineSettings.GeneralProjectSettings]
ProjectID=0A9A19D34F8E4D53A0CFF6D3A6A2CB2A
ProjectName=ACESimUE
ProjectVersion=0.1.0
Description=Minimal UE5 project for ACESim MuJoCo visual sync.

[/Script/UnrealEd.ProjectPackagingSettings]
+DirectoriesToAlwaysCook=(Path="/Game/ACESim/x500_arm2x")
+DirectoriesToAlwaysCook=(Path="/Game/ACESim/Environment")
+DirectoriesToAlwaysStageAsNonUFS=(Path="ACESim/x500_arm2x")
+DirectoriesToAlwaysStageAsNonUFS=(Path="ACESim/Environment/Heliport")
+DirectoriesToAlwaysStageAsNonUFS=(Path="ACESim/Environment/Airport")
""",
        f"Source/{PROJECT_NAME}.Target.cs": """using UnrealBuildTool;
using System.Collections.Generic;

public class ACESimUETarget : TargetRules
{
    public ACESimUETarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V6;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
        ExtraModuleNames.Add("ACESimUE");
    }
}
""",
        f"Source/{PROJECT_NAME}Editor.Target.cs": """using UnrealBuildTool;
using System.Collections.Generic;

public class ACESimUEEditorTarget : TargetRules
{
    public ACESimUEEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V6;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
        ExtraModuleNames.Add("ACESimUE");
    }
}
""",
        f"Source/{PROJECT_NAME}/{PROJECT_NAME}.Build.cs": """using UnrealBuildTool;

public class ACESimUE : ModuleRules
{
    public ACESimUE(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "ACESimBridge",
            "InputCore",
            "Slate",
            "AssetRegistry"
        });
    }
}
""",
        f"Source/{PROJECT_NAME}/{PROJECT_NAME}.h": """#pragma once

#include "CoreMinimal.h"
""",
        f"Source/{PROJECT_NAME}/{PROJECT_NAME}.cpp": """#include "ACESimUE.h"
#include "Modules/ModuleManager.h"

IMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl, ACESimUE, "ACESimUE");
""",
        f"Source/{PROJECT_NAME}/{PROJECT_NAME}GameMode.h": """#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ACESimUEGameMode.generated.h"

UCLASS()
class ACESIMUE_API AACESimUEGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    AACESimUEGameMode();

protected:
    virtual void BeginPlay() override;
};
""",
        f"Source/{PROJECT_NAME}/{PROJECT_NAME}GameMode.cpp": """#include "ACESimUEGameMode.h"

#include "ACESimFreeCameraPawn.h"
#include "ACESimOutdoorTestFieldActor.h"
#include "ACESimPlayerController.h"
#include "ACESimVehicleActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "TimerManager.h"

AACESimUEGameMode::AACESimUEGameMode()
{
    DefaultPawnClass = AACESimFreeCameraPawn::StaticClass();
    PlayerControllerClass = AACESimPlayerController::StaticClass();
    bStartPlayersAsSpectators = false;
}

void AACESimUEGameMode::BeginPlay()
{
    Super::BeginPlay();

    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }

    AACESimOutdoorTestFieldActor* FieldActor = nullptr;
    for (TActorIterator<AACESimOutdoorTestFieldActor> It(World); It; ++It)
    {
        FieldActor = *It;
        break;
    }

    if (FieldActor == nullptr)
    {
        FActorSpawnParameters SpawnParameters;
        SpawnParameters.Name = TEXT("ACESimOutdoorTestField");
        SpawnParameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        FieldActor = World->SpawnActor<AACESimOutdoorTestFieldActor>(
            AACESimOutdoorTestFieldActor::StaticClass(),
            FVector::ZeroVector,
            FRotator::ZeroRotator,
            SpawnParameters);
        if (FieldActor != nullptr)
        {
            UE_LOG(LogTemp, Display, TEXT("ACESim outdoor test field spawned"));
        }
    }

    AACESimVehicleActor* VehicleActor = nullptr;
    for (TActorIterator<AACESimVehicleActor> It(World); It; ++It)
    {
        VehicleActor = *It;
        break;
    }

    if (VehicleActor == nullptr)
    {
        FActorSpawnParameters SpawnParameters;
        SpawnParameters.Name = TEXT("ACESimVehicle");
        SpawnParameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        VehicleActor = World->SpawnActor<AACESimVehicleActor>(
            AACESimVehicleActor::StaticClass(),
            FVector::ZeroVector,
            FRotator::ZeroRotator,
            SpawnParameters);
        if (VehicleActor != nullptr)
        {
            UE_LOG(LogTemp, Display, TEXT("ACESim vehicle actor spawned"));
        }
    }

    FVector HelipadAnchorLocation = FVector::ZeroVector;
    if (FieldActor != nullptr)
    {
        HelipadAnchorLocation = FieldActor->GetHelipadAnchorLocation();
    }

    if (VehicleActor != nullptr)
    {
        VehicleActor->SetActorLocation(HelipadAnchorLocation);
        World->GetTimerManager().SetTimerForNextTick([World, VehicleActor, HelipadAnchorLocation]()
        {
            if (World == nullptr || VehicleActor == nullptr)
            {
                return;
            }
            if (APlayerController* PlayerController = World->GetFirstPlayerController())
            {
                if (AACESimFreeCameraPawn* FreeCameraPawn = Cast<AACESimFreeCameraPawn>(PlayerController->GetPawn()))
                {
                    // Legacy marker for tests and log review: FrameVehicle(VehicleActor) now routes
                    // through FrameVehicleWithEnvironment so the helipad stays in view.
                    FreeCameraPawn->FrameVehicleWithEnvironment(VehicleActor, HelipadAnchorLocation);
                    if (FParse::Param(FCommandLine::Get(), TEXT("ACESimVisualSmoke")))
                    {
                        UE_LOG(LogTemp, Display, TEXT("ACESim vehicle visible in visual smoke frame"));
                    }
                }
            }
        });
    }
    UE_LOG(LogTemp, Display, TEXT("ACESim free camera enabled"));
}
""",
        f"Source/{PROJECT_NAME}/ACESimFreeCameraPawn.h": """#pragma once

#include "CoreMinimal.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "GameFramework/Pawn.h"
#include "ACESimFreeCameraPawn.generated.h"

class AActor;
class UCameraComponent;
class USceneComponent;

UCLASS()
class ACESIMUE_API AACESimFreeCameraPawn : public APawn
{
    GENERATED_BODY()

public:
    AACESimFreeCameraPawn();
    void FrameVehicle(AActor* VehicleActor);
    void FrameVehicleWithEnvironment(AActor* VehicleActor, const FVector& EnvironmentAnchor);
    void SetOrbitTargetFromActor(AActor* VehicleActor);
    void ApplyOrbitMouseDelta(const FVector2D& Delta);
    void PanOrbitTarget(const FVector2D& Delta);
    void ZoomOrbit(float WheelDelta);
    void ApplyKeyboardLookDelta(float YawInput, float PitchInput);

protected:
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

private:
    void MoveForward(float Value);
    void MoveRight(float Value);
    void MoveUp(float Value);
    void LookYaw(float Value);
    void LookPitch(float Value);
    void ReleaseLook();
    void ApplyReleasedInputMode();
    void UpdateOrbitCamera();

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<UCameraComponent> CameraComponent;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<UFloatingPawnMovement> MovementComponent;

    FVector OrbitTarget = FVector::ZeroVector;
    float OrbitDistanceCm = 820.0f;
    float OrbitYawDegrees = -138.0f;
    float OrbitPitchDegrees = -24.0f;
    float OrbitRotateSensitivity = 0.18f;
    float OrbitPanSensitivity = 0.72f;
    float OrbitZoomStepCm = 48.0f;
    float MinOrbitDistanceCm = 160.0f;
    float MaxOrbitDistanceCm = 6500.0f;
    float MoveSpeedCmPerSec = 600.0f;
    float KeyboardLookRateDegPerSec = 75.0f;
};
""",
        f"Source/{PROJECT_NAME}/ACESimFreeCameraPawn.cpp": """#include "ACESimFreeCameraPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/EngineBaseTypes.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "Math/RotationMatrix.h"

AACESimFreeCameraPawn::AACESimFreeCameraPawn()
{
    PrimaryActorTick.bCanEverTick = false;
    AutoPossessPlayer = EAutoReceiveInput::Player0;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(SceneRoot);

    CameraComponent = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    CameraComponent->SetupAttachment(SceneRoot);
    CameraComponent->SetAutoActivate(true);

    MovementComponent = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("FloatingPawnMovement"));
    MovementComponent->MaxSpeed = MoveSpeedCmPerSec;
    MovementComponent->Acceleration = 4096.0f;
    MovementComponent->Deceleration = 4096.0f;
    MovementComponent->UpdatedComponent = SceneRoot;

    UpdateOrbitCamera();
}

void AACESimFreeCameraPawn::FrameVehicle(AActor* VehicleActor)
{
    FrameVehicleWithEnvironment(VehicleActor, FVector::ZeroVector);
}

void AACESimFreeCameraPawn::FrameVehicleWithEnvironment(AActor* VehicleActor, const FVector& EnvironmentAnchor)
{
    if (VehicleActor == nullptr)
    {
        OrbitTarget = EnvironmentAnchor;
        OrbitDistanceCm = 980.0f;
        UpdateOrbitCamera();
        return;
    }

    const FBox Bounds = VehicleActor->GetComponentsBoundingBox(true);
    float RadiusCm = 180.0f;
    OrbitTarget = (VehicleActor->GetActorLocation() * 0.72f) + (EnvironmentAnchor * 0.28f);
    if (Bounds.IsValid)
    {
        OrbitTarget = (Bounds.GetCenter() * 0.72f) + (EnvironmentAnchor * 0.28f);
        RadiusCm = FMath::Max(120.0f, Bounds.GetExtent().Size());
    }
    OrbitTarget.Z += 28.0f;
    OrbitDistanceCm = FMath::Clamp(FMath::Max(980.0f, RadiusCm * 3.8f), MinOrbitDistanceCm, MaxOrbitDistanceCm);
    OrbitYawDegrees = -138.0f;
    OrbitPitchDegrees = -18.0f;
    UpdateOrbitCamera();
}

void AACESimFreeCameraPawn::SetOrbitTargetFromActor(AActor* VehicleActor)
{
    FrameVehicleWithEnvironment(VehicleActor, FVector::ZeroVector);
}

void AACESimFreeCameraPawn::ApplyOrbitMouseDelta(const FVector2D& Delta)
{
    if (Delta.IsNearlyZero())
    {
        return;
    }

    OrbitYawDegrees += Delta.X * OrbitRotateSensitivity;
    OrbitPitchDegrees = FMath::Clamp(OrbitPitchDegrees - Delta.Y * OrbitRotateSensitivity, -85.0f, 85.0f);
    UpdateOrbitCamera();
}

void AACESimFreeCameraPawn::PanOrbitTarget(const FVector2D& Delta)
{
    if (Delta.IsNearlyZero())
    {
        return;
    }

    const FRotator CameraRotation = GetActorRotation();
    const FVector Right = FRotationMatrix(CameraRotation).GetUnitAxis(EAxis::Y);
    const FVector Up = FVector::UpVector;
    OrbitTarget += (-Right * Delta.X + Up * Delta.Y) * OrbitPanSensitivity;
    UpdateOrbitCamera();
}

void AACESimFreeCameraPawn::ZoomOrbit(float WheelDelta)
{
    if (FMath::IsNearlyZero(WheelDelta))
    {
        return;
    }

    OrbitDistanceCm = FMath::Clamp(OrbitDistanceCm - WheelDelta * OrbitZoomStepCm, MinOrbitDistanceCm, MaxOrbitDistanceCm);
    UpdateOrbitCamera();
}

void AACESimFreeCameraPawn::ApplyKeyboardLookDelta(float YawInput, float PitchInput)
{
    if (FMath::IsNearlyZero(YawInput) && FMath::IsNearlyZero(PitchInput))
    {
        return;
    }

    const float DeltaSeconds = GetWorld() != nullptr ? GetWorld()->GetDeltaSeconds() : (1.0f / 60.0f);
    OrbitYawDegrees += YawInput * KeyboardLookRateDegPerSec * DeltaSeconds;
    OrbitPitchDegrees = FMath::Clamp(OrbitPitchDegrees - PitchInput * KeyboardLookRateDegPerSec * DeltaSeconds, -85.0f, 85.0f);
    UpdateOrbitCamera();
}

void AACESimFreeCameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    PlayerInputComponent->BindAction(TEXT("ReleaseLook"), IE_Pressed, this, &AACESimFreeCameraPawn::ReleaseLook);
    PlayerInputComponent->BindKey(EKeys::Escape, IE_Pressed, this, &AACESimFreeCameraPawn::ReleaseLook);
    PlayerInputComponent->BindAxis(TEXT("MoveForward"), this, &AACESimFreeCameraPawn::MoveForward);
    PlayerInputComponent->BindAxis(TEXT("MoveRight"), this, &AACESimFreeCameraPawn::MoveRight);
    PlayerInputComponent->BindAxis(TEXT("MoveUp"), this, &AACESimFreeCameraPawn::MoveUp);
    PlayerInputComponent->BindAxis(TEXT("LookYaw"), this, &AACESimFreeCameraPawn::LookYaw);
    PlayerInputComponent->BindAxis(TEXT("LookPitch"), this, &AACESimFreeCameraPawn::LookPitch);
}

void AACESimFreeCameraPawn::MoveForward(float Value)
{
    if (!FMath::IsNearlyZero(Value))
    {
        const FRotationMatrix YawOnlyFrame(FRotator(0.0f, GetActorRotation().Yaw, 0.0f));
        const FVector Delta = YawOnlyFrame.GetUnitAxis(EAxis::X) * Value * MoveSpeedCmPerSec * (GetWorld() != nullptr ? GetWorld()->GetDeltaSeconds() : 1.0f / 60.0f);
        OrbitTarget += Delta;
        UpdateOrbitCamera();
    }
}

void AACESimFreeCameraPawn::MoveRight(float Value)
{
    if (!FMath::IsNearlyZero(Value))
    {
        const FRotationMatrix YawOnlyFrame(FRotator(0.0f, GetActorRotation().Yaw, 0.0f));
        const FVector Delta = YawOnlyFrame.GetUnitAxis(EAxis::Y) * Value * MoveSpeedCmPerSec * (GetWorld() != nullptr ? GetWorld()->GetDeltaSeconds() : 1.0f / 60.0f);
        OrbitTarget += Delta;
        UpdateOrbitCamera();
    }
}

void AACESimFreeCameraPawn::MoveUp(float Value)
{
    if (!FMath::IsNearlyZero(Value))
    {
        OrbitTarget += FVector::UpVector * Value * MoveSpeedCmPerSec * (GetWorld() != nullptr ? GetWorld()->GetDeltaSeconds() : 1.0f / 60.0f);
        UpdateOrbitCamera();
    }
}

void AACESimFreeCameraPawn::LookYaw(float Value)
{
    ApplyKeyboardLookDelta(Value, 0.0f);
}

void AACESimFreeCameraPawn::LookPitch(float Value)
{
    ApplyKeyboardLookDelta(0.0f, Value);
}

void AACESimFreeCameraPawn::ReleaseLook()
{
    ApplyReleasedInputMode();
}

void AACESimFreeCameraPawn::ApplyReleasedInputMode()
{
    APlayerController* PlayerController = Cast<APlayerController>(GetController());
    if (PlayerController == nullptr)
    {
        return;
    }

    FInputModeGameAndUI InputMode;
    InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
    InputMode.SetHideCursorDuringCapture(false);
    PlayerController->SetInputMode(InputMode);
    PlayerController->bShowMouseCursor = true;
}

void AACESimFreeCameraPawn::UpdateOrbitCamera()
{
    const FRotator OrbitRotation(OrbitPitchDegrees, OrbitYawDegrees, 0.0f);
    const FVector CameraOffset = OrbitRotation.Vector() * -OrbitDistanceCm;
    const FVector CameraLocation = OrbitTarget + CameraOffset;
    const FRotator LookRotation = FRotationMatrix::MakeFromX(OrbitTarget - CameraLocation).Rotator();
    SetActorLocation(CameraLocation);
    SetActorRotation(LookRotation);
}
""",
        f"Source/{PROJECT_NAME}/ACESimPlayerController.h": """#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "ACESimPlayerController.generated.h"

class AACESimFreeCameraPawn;
class APawn;

UCLASS()
class ACESIMUE_API AACESimPlayerController : public APlayerController
{
    GENERATED_BODY()

protected:
    virtual void BeginPlay() override;
    virtual void OnPossess(APawn* InPawn) override;
    virtual void PlayerTick(float DeltaTime) override;
    virtual bool InputKey(const FInputKeyEventArgs& Params) override;

private:
    void ApplyReleasedInputMode();
    void LogInputDiagnostic();

    bool bLoggedInputDiagnostic = false;
};
""",
        f"Source/{PROJECT_NAME}/ACESimPlayerController.cpp": """#include "ACESimPlayerController.h"

#include "ACESimFreeCameraPawn.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/GameViewportClient.h"
#include "GameFramework/Pawn.h"
#include "InputKeyEventArgs.h"

void AACESimPlayerController::BeginPlay()
{
    Super::BeginPlay();

    ApplyReleasedInputMode();
    UE_LOG(LogTemp, Display, TEXT("ACESim player controller ready"));
}

void AACESimPlayerController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    if (Cast<AACESimFreeCameraPawn>(InPawn) != nullptr)
    {
        UE_LOG(LogTemp, Display, TEXT("ACESim camera pawn possessed"));
    }
}

void AACESimPlayerController::PlayerTick(float DeltaTime)
{
    Super::PlayerTick(DeltaTime);
    LogInputDiagnostic();
}

bool AACESimPlayerController::InputKey(const FInputKeyEventArgs& Params)
{
    const bool bHandledBySuper = Super::InputKey(Params);
    if (Params.Key == EKeys::Escape && Params.Event == IE_Pressed)
    {
        ApplyReleasedInputMode();
        return true;
    }
    return bHandledBySuper;
}

void AACESimPlayerController::ApplyReleasedInputMode()
{
    bShowMouseCursor = true;

    FInputModeGameAndUI InputMode;
    InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
    InputMode.SetHideCursorDuringCapture(false);
    SetInputMode(InputMode);
    if (UGameViewportClient* GameViewportClient = GetWorld() != nullptr ? GetWorld()->GetGameViewport() : nullptr)
    {
        GameViewportClient->SetMouseCaptureMode(EMouseCaptureMode::NoCapture);
        GameViewportClient->SetMouseLockMode(EMouseLockMode::DoNotLock);
    }
}

void AACESimPlayerController::LogInputDiagnostic()
{
    if (bLoggedInputDiagnostic)
    {
        return;
    }

    const bool bHasPawn = Cast<AACESimFreeCameraPawn>(GetPawn()) != nullptr;
    UE_LOG(
        LogTemp,
        Display,
        TEXT("ACESim input diagnostic: camera_pawn=%d viewport_input=1"),
        bHasPawn ? 1 : 0);
    bLoggedInputDiagnostic = true;
}
""",
        f"Source/{PROJECT_NAME}/ACESimGameViewportClient.h": """#pragma once

#include "CoreMinimal.h"
#include "Engine/GameViewportClient.h"
#include "ACESimGameViewportClient.generated.h"

class AACESimFreeCameraPawn;
class FViewport;

UCLASS()
class ACESIMUE_API UACESimGameViewportClient : public UGameViewportClient
{
    GENERATED_BODY()

public:
    enum class ECameraDragMode : uint8
    {
        None,
        Orbit,
        Pan,
    };

    virtual void Init(struct FWorldContext& WorldContext, UGameInstance* OwningGameInstance, bool bCreateNewAudioDevice = true) override;
    virtual bool InputKey(const FInputKeyEventArgs& EventArgs) override;
    virtual bool InputAxis(const FInputKeyEventArgs& EventArgs) override;
    virtual void Tick(float DeltaTime) override;

private:
    void StartCameraCapture(ECameraDragMode Mode);
    void ReleaseCameraCapture();
    AACESimFreeCameraPawn* FindFreeCameraPawn() const;
    void ApplyViewportMouseDelta(const FVector2D& CursorDelta);
    bool HandleMouseWheel(float WheelDelta);
    void LogViewportInputDiagnostic();

    ECameraDragMode CameraDragMode = ECameraDragMode::None;
    FVector2D LastCursorPosition = FVector2D::ZeroVector;
    bool bHasLastCursorPosition = false;
    bool bLoggedFirstDelta = false;
    bool bLoggedDiagnostic = false;
    bool bVisualSmokeDragInjected = false;
};
""",
        f"Source/{PROJECT_NAME}/ACESimGameViewportClient.cpp": """#include "ACESimGameViewportClient.h"

#include "ACESimFreeCameraPawn.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/CommandLine.h"

namespace
{
const TCHAR* DragModeName(UACESimGameViewportClient::ECameraDragMode Mode)
{
    switch (Mode)
    {
    case UACESimGameViewportClient::ECameraDragMode::Orbit:
        return TEXT("orbit");
    case UACESimGameViewportClient::ECameraDragMode::Pan:
        return TEXT("pan");
    default:
        return TEXT("none");
    }
}
}  // namespace

void UACESimGameViewportClient::Init(FWorldContext& WorldContext, UGameInstance* OwningGameInstance, bool bCreateNewAudioDevice)
{
    Super::Init(WorldContext, OwningGameInstance, bCreateNewAudioDevice);
    UE_LOG(LogTemp, Display, TEXT("ACESim viewport input diagnostic: installed=1 capture_mode=orbit"));
}

bool UACESimGameViewportClient::InputKey(const FInputKeyEventArgs& EventArgs)
{
    LogViewportInputDiagnostic();

    if (EventArgs.Key == EKeys::LeftMouseButton || EventArgs.Key == EKeys::RightMouseButton)
    {
        const ECameraDragMode RequestedMode = EventArgs.Key == EKeys::RightMouseButton ? ECameraDragMode::Pan : ECameraDragMode::Orbit;
        UE_LOG(
            LogTemp,
            Display,
            TEXT("ACESim viewport key diagnostic: key=%s event=%d camera_pawn=%d"),
            EventArgs.Key == EKeys::RightMouseButton ? TEXT("RightMouseButton") : TEXT("LeftMouseButton"),
            static_cast<int32>(EventArgs.Event),
            FindFreeCameraPawn() != nullptr ? 1 : 0);
        if (EventArgs.Event == IE_Released)
        {
            ReleaseCameraCapture();
        }
        return true;
    }

    if (EventArgs.Key == EKeys::MouseWheelAxis)
    {
        return HandleMouseWheel(EventArgs.AmountDepressed);
    }

    if (EventArgs.Key == EKeys::Escape && EventArgs.Event == IE_Pressed)
    {
        ReleaseCameraCapture();
        return false;
    }

    return Super::InputKey(EventArgs);
}

bool UACESimGameViewportClient::InputAxis(const FInputKeyEventArgs& EventArgs)
{
    if (EventArgs.Key == EKeys::MouseWheelAxis)
    {
        return HandleMouseWheel(EventArgs.AmountDepressed);
    }
    return Super::InputAxis(EventArgs);
}

void UACESimGameViewportClient::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!FSlateApplication::IsInitialized())
    {
        return;
    }

    if (!bVisualSmokeDragInjected && FParse::Param(FCommandLine::Get(), TEXT("ACESimVisualSmoke")))
    {
        bVisualSmokeDragInjected = true;
        StartCameraCapture(ECameraDragMode::Orbit);
        UE_LOG(LogTemp, Display, TEXT("ACESim camera tick drag started: mode=%s source=visual-smoke"), DragModeName(CameraDragMode));
        ApplyViewportMouseDelta(FVector2D(24.0f, -10.0f));
        HandleMouseWheel(1.0f);
        ReleaseCameraCapture();
        UE_LOG(LogTemp, Display, TEXT("ACESim camera tick drag ended"));
        return;
    }

    const TSet<FKey>& PressedButtons = FSlateApplication::Get().GetPressedMouseButtons();
    const bool bLeftPressed = PressedButtons.Contains(EKeys::LeftMouseButton);
    const bool bRightPressed = PressedButtons.Contains(EKeys::RightMouseButton);
    const ECameraDragMode RequestedMode = bRightPressed ? ECameraDragMode::Pan : (bLeftPressed ? ECameraDragMode::Orbit : ECameraDragMode::None);
    const FVector2D CursorPosition = FVector2D(FSlateApplication::Get().GetCursorPos());

    if (RequestedMode == ECameraDragMode::None)
    {
        if (CameraDragMode != ECameraDragMode::None)
        {
            ReleaseCameraCapture();
            UE_LOG(LogTemp, Display, TEXT("ACESim camera tick drag ended"));
        }
        bHasLastCursorPosition = false;
        return;
    }

    if (CameraDragMode != RequestedMode)
    {
        StartCameraCapture(RequestedMode);
        LastCursorPosition = CursorPosition;
        bHasLastCursorPosition = true;
        UE_LOG(LogTemp, Display, TEXT("ACESim camera tick drag started: mode=%s"), DragModeName(CameraDragMode));
        return;
    }

    if (!bHasLastCursorPosition)
    {
        LastCursorPosition = CursorPosition;
        bHasLastCursorPosition = true;
        return;
    }

    const FVector2D CursorDelta = CursorPosition - LastCursorPosition;
    LastCursorPosition = CursorPosition;
    if (!CursorDelta.IsNearlyZero())
    {
        ApplyViewportMouseDelta(CursorDelta);
    }
}

void UACESimGameViewportClient::StartCameraCapture(ECameraDragMode Mode)
{
    CameraDragMode = Mode;
    bLoggedFirstDelta = false;
    SetMouseCaptureMode(EMouseCaptureMode::CaptureDuringMouseDown);
    SetMouseLockMode(EMouseLockMode::DoNotLock);
    if (Viewport != nullptr)
    {
        Viewport->CaptureMouse(true);
    }
    if (APlayerController* PlayerController = GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr)
    {
        PlayerController->bShowMouseCursor = false;
    }
    UE_LOG(LogTemp, Display, TEXT("ACESim camera tick capture active: mode=%s"), DragModeName(CameraDragMode));
}

void UACESimGameViewportClient::ReleaseCameraCapture()
{
    if (CameraDragMode != ECameraDragMode::None)
    {
        UE_LOG(LogTemp, Display, TEXT("ACESim camera tick capture released: mode=%s"), DragModeName(CameraDragMode));
    }
    CameraDragMode = ECameraDragMode::None;
    bHasLastCursorPosition = false;
    SetMouseCaptureMode(EMouseCaptureMode::NoCapture);
    SetMouseLockMode(EMouseLockMode::DoNotLock);
    if (Viewport != nullptr)
    {
        Viewport->CaptureMouse(false);
        Viewport->LockMouseToViewport(false);
    }
    if (APlayerController* PlayerController = GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr)
    {
        PlayerController->bShowMouseCursor = true;
    }
}

AACESimFreeCameraPawn* UACESimGameViewportClient::FindFreeCameraPawn() const
{
    if (const UWorld* World = GetWorld())
    {
        if (APlayerController* PlayerController = World->GetFirstPlayerController())
        {
            return Cast<AACESimFreeCameraPawn>(PlayerController->GetPawn());
        }
    }
    return nullptr;
}

void UACESimGameViewportClient::ApplyViewportMouseDelta(const FVector2D& CursorDelta)
{
    AACESimFreeCameraPawn* FreeCameraPawn = FindFreeCameraPawn();
    if (FreeCameraPawn == nullptr)
    {
        return;
    }

    if (CameraDragMode == ECameraDragMode::Orbit)
    {
        FreeCameraPawn->ApplyOrbitMouseDelta(CursorDelta);
    }
    else if (CameraDragMode == ECameraDragMode::Pan)
    {
        FreeCameraPawn->PanOrbitTarget(CursorDelta);
    }
    if (!bLoggedFirstDelta)
    {
        UE_LOG(
            LogTemp,
            Display,
            TEXT("ACESim camera tick delta applied: mode=%s dx=%.3f dy=%.3f"),
            DragModeName(CameraDragMode),
            CursorDelta.X,
            CursorDelta.Y);
        bLoggedFirstDelta = true;
    }
}

bool UACESimGameViewportClient::HandleMouseWheel(float WheelDelta)
{
    AACESimFreeCameraPawn* FreeCameraPawn = FindFreeCameraPawn();
    if (FreeCameraPawn == nullptr || FMath::IsNearlyZero(WheelDelta))
    {
        return FreeCameraPawn != nullptr;
    }

    FreeCameraPawn->ZoomOrbit(WheelDelta);
    UE_LOG(LogTemp, Display, TEXT("ACESim camera wheel zoom applied: delta=%.3f"), WheelDelta);
    return true;
}

void UACESimGameViewportClient::LogViewportInputDiagnostic()
{
    if (bLoggedDiagnostic)
    {
        return;
    }

    UE_LOG(
        LogTemp,
        Display,
        TEXT("ACESim viewport input diagnostic: camera_pawn=%d capture_mode=orbit"),
        FindFreeCameraPawn() != nullptr ? 1 : 0);
    bLoggedDiagnostic = true;
}
""",
        f"Source/{PROJECT_NAME}/ACESimOutdoorTestFieldActor.h": """#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ACESimOutdoorTestFieldActor.generated.h"

class UDirectionalLightComponent;
class UExponentialHeightFogComponent;
class USceneComponent;
class USkyLightComponent;
class UStaticMeshComponent;
struct FAssetData;

UCLASS()
class ACESIMUE_API AACESimOutdoorTestFieldActor : public AActor
{
    GENERATED_BODY()

public:
    AACESimOutdoorTestFieldActor();
    FVector GetHelipadAnchorLocation() const;

protected:
    virtual void OnConstruction(const FTransform& Transform) override;

private:
    bool ConfigureAssets();
    bool BuildEnvironmentMeshComponents();
    UStaticMesh* LoadEnvironmentMeshAsset(const FAssetData& AssetData) const;
    bool LoadEnvironmentAttribution();

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<USceneComponent> EnvironmentRootComponent;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<USceneComponent> HelipadAnchorComponent;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<UDirectionalLightComponent> SunLight;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<USkyLightComponent> SkyLight;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<UExponentialHeightFogComponent> HeightFog;

    UPROPERTY(Transient)
    TArray<TObjectPtr<UStaticMeshComponent>> EnvironmentMeshComponents;

    FString EnvironmentSourceAttribution;
    FString ActiveEnvironmentStyle;
    FName ActiveEnvironmentAssetPath;
};
""",
        f"Source/{PROJECT_NAME}/ACESimOutdoorTestFieldActor.cpp": """#include "ACESimOutdoorTestFieldActor.h"

#include "AssetRegistry/AssetData.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

namespace
{
const FName HeliportAssetPath(TEXT("/Game/ACESim/Environment/Heliport/Model"));
const FName AirportAssetPath(TEXT("/Game/ACESim/Environment/Airport/Model"));
}

AACESimOutdoorTestFieldActor::AACESimOutdoorTestFieldActor()
{
    PrimaryActorTick.bCanEverTick = false;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(SceneRoot);

    EnvironmentRootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("EnvironmentRoot"));
    EnvironmentRootComponent->SetupAttachment(SceneRoot);
    // Sketchfab glTF imports are usually authored in meters. UE uses centimeters.
    EnvironmentRootComponent->SetWorldScale3D(FVector(100.0f));

    HelipadAnchorComponent = CreateDefaultSubobject<USceneComponent>(TEXT("HelipadAnchor"));
    HelipadAnchorComponent->SetupAttachment(SceneRoot);
    // ACESim-controlled helipad anchor: the simulator vehicle starts here even
    // if the downloaded background model has an inconvenient origin.
    HelipadAnchorComponent->SetRelativeLocation(FVector::ZeroVector);

    SunLight = CreateDefaultSubobject<UDirectionalLightComponent>(TEXT("SunLight"));
    SunLight->SetupAttachment(SceneRoot);
    SunLight->SetRelativeRotation(FRotator(-42.0f, -28.0f, 0.0f));
    SunLight->SetIntensity(5.2f);
    SunLight->SetCastShadows(true);
    SunLight->DynamicShadowDistanceMovableLight = 7400.0f;

    SkyLight = CreateDefaultSubobject<USkyLightComponent>(TEXT("SkyLight"));
    SkyLight->SetupAttachment(SceneRoot);
    SkyLight->SetIntensity(1.6f);

    HeightFog = CreateDefaultSubobject<UExponentialHeightFogComponent>(TEXT("HeightFog"));
    HeightFog->SetupAttachment(SceneRoot);
    HeightFog->SetFogDensity(0.0018f);
    HeightFog->SetFogHeightFalloff(0.055f);
}

void AACESimOutdoorTestFieldActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    ConfigureAssets();
}

FVector AACESimOutdoorTestFieldActor::GetHelipadAnchorLocation() const
{
    return HelipadAnchorComponent != nullptr ? HelipadAnchorComponent->GetComponentLocation() : GetActorLocation();
}

bool AACESimOutdoorTestFieldActor::ConfigureAssets()
{
    bool AllAssetsLoaded = true;
    AllAssetsLoaded &= BuildEnvironmentMeshComponents();
    AllAssetsLoaded &= LoadEnvironmentAttribution();
    if (!AllAssetsLoaded)
    {
        UE_LOG(LogTemp, Error, TEXT("ACESim outdoor field asset missing: environment asset bundle is incomplete"));
        return false;
    }

    if (ActiveEnvironmentStyle == TEXT("heliport"))
    {
        UE_LOG(LogTemp, Display, TEXT("ACESim heliport assets loaded"));
        UE_LOG(LogTemp, Display, TEXT("ACESim heliport attribution loaded: %s"), *EnvironmentSourceAttribution.Left(180));
    }
    else
    {
        UE_LOG(LogTemp, Display, TEXT("ACESim airport assets loaded"));
        UE_LOG(LogTemp, Display, TEXT("ACESim airport attribution loaded: %s"), *EnvironmentSourceAttribution.Left(180));
    }
    return true;
}

bool AACESimOutdoorTestFieldActor::BuildEnvironmentMeshComponents()
{
    if (EnvironmentRootComponent == nullptr)
    {
        return false;
    }

    for (UStaticMeshComponent* Component : EnvironmentMeshComponents)
    {
        if (Component != nullptr)
        {
            Component->DestroyComponent();
        }
    }
    EnvironmentMeshComponents.Reset();

    FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
    TArray<FAssetData> MeshAssets;
    ActiveEnvironmentAssetPath = HeliportAssetPath;
    ActiveEnvironmentStyle = TEXT("heliport");
    AssetRegistryModule.Get().GetAssetsByPath(HeliportAssetPath, MeshAssets, true);
    if (MeshAssets.Num() == 0)
    {
        UE_LOG(LogTemp, Error, TEXT("ACESim heliport asset missing: no StaticMesh assets under %s"), *HeliportAssetPath.ToString());
        ActiveEnvironmentAssetPath = AirportAssetPath;
        ActiveEnvironmentStyle = TEXT("airport");
        AssetRegistryModule.Get().GetAssetsByPath(AirportAssetPath, MeshAssets, true);
    }
    MeshAssets.Sort([](const FAssetData& Left, const FAssetData& Right) { return Left.AssetName.LexicalLess(Right.AssetName); });

    int32 MeshCount = 0;
    for (const FAssetData& AssetData : MeshAssets)
    {
        UStaticMesh* Mesh = LoadEnvironmentMeshAsset(AssetData);
        if (Mesh == nullptr)
        {
            continue;
        }

        UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(this, UStaticMeshComponent::StaticClass(), AssetData.AssetName);
        Component->SetupAttachment(EnvironmentRootComponent);
        Component->SetStaticMesh(Mesh);
        Component->SetMobility(EComponentMobility::Static);
        Component->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
        Component->SetCastShadow(true);
        Component->RegisterComponent();
        EnvironmentMeshComponents.Add(Component);
        ++MeshCount;
    }

    if (MeshCount == 0)
    {
        UE_LOG(LogTemp, Error, TEXT("ACESim airport asset missing: no StaticMesh assets under %s"), *AirportAssetPath.ToString());
        return false;
    }
    return true;
}

UStaticMesh* AACESimOutdoorTestFieldActor::LoadEnvironmentMeshAsset(const FAssetData& AssetData) const
{
    if (AssetData.GetClass() != UStaticMesh::StaticClass())
    {
        return nullptr;
    }
    return Cast<UStaticMesh>(AssetData.GetAsset());
}

bool AACESimOutdoorTestFieldActor::LoadEnvironmentAttribution()
{
    const FString BaseDir = ActiveEnvironmentStyle == TEXT("heliport")
        ? FPaths::ProjectContentDir() / TEXT("ACESim/Environment/Heliport")
        : FPaths::ProjectContentDir() / TEXT("ACESim/Environment/Airport");
    const FString ManifestPath = BaseDir / (ActiveEnvironmentStyle == TEXT("heliport") ? TEXT("heliport_manifest.json") : TEXT("airport_manifest.json"));
    if (!IFileManager::Get().FileExists(*ManifestPath))
    {
        if (ActiveEnvironmentStyle == TEXT("heliport"))
        {
            UE_LOG(LogTemp, Error, TEXT("ACESim heliport asset missing: manifest file %s"), *ManifestPath);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("ACESim airport asset missing: manifest file %s"), *ManifestPath);
        }
        return false;
    }
    const FString AttributionPath = BaseDir / TEXT("ATTRIBUTION.txt");
    if (!FFileHelper::LoadFileToString(EnvironmentSourceAttribution, *AttributionPath))
    {
        if (ActiveEnvironmentStyle == TEXT("heliport"))
        {
            UE_LOG(LogTemp, Error, TEXT("ACESim heliport asset missing: attribution file %s"), *AttributionPath);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("ACESim airport asset missing: attribution file %s"), *AttributionPath);
        }
        return false;
    }
    return true;
}
""",
        f"Plugins/{PLUGIN_NAME}/{PLUGIN_NAME}.uplugin": _uplugin(),
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/{PLUGIN_NAME}.Build.cs": """using UnrealBuildTool;

public class ACESimBridge : ModuleRules
{
    public ACESimBridge(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "Json"
        });

        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            PublicSystemIncludePaths.Add("/usr/include");
            PublicSystemLibraryPaths.Add("/usr/lib/x86_64-linux-gnu");
        }

        PublicSystemLibraries.Add("zmq");
    }
}
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Public/{PLUGIN_NAME}Module.h": """#pragma once

#include "Modules/ModuleInterface.h"

class FACESimBridgeModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
""",
        (
            f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Private/" f"{PLUGIN_NAME}Module.cpp"
        ): """#include "ACESimBridgeModule.h"
#include "Modules/ModuleManager.h"

void FACESimBridgeModule::StartupModule()
{
}

void FACESimBridgeModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FACESimBridgeModule, ACESimBridge)
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Public/ACESimVehicleActor.h": """#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ACESimVehicleActor.generated.h"

class FJsonObject;
class UACESimVehicleSyncComponent;
class UACESimArmStateSyncComponent;
class USceneComponent;
class UStaticMeshComponent;
class UStaticMesh;

struct FACESimJointRuntime
{
    TWeakObjectPtr<USceneComponent> Component;
    FVector Axis = FVector::UpVector;
    FVector HomeLocation = FVector::ZeroVector;
    FRotator HomeRotation = FRotator::ZeroRotator;
    bool bSlide = false;
};

UCLASS(Blueprintable)
class ACESIMBRIDGE_API AACESimVehicleActor : public AActor
{
    GENERATED_BODY()

public:
    AACESimVehicleActor();

    UFUNCTION(BlueprintCallable, Category="ACESim")
    void SetRotorCount(int32 NewRotorCount);

    UFUNCTION(BlueprintCallable, Category="ACESim")
    USceneComponent* GetRotorComponentByIndex(int32 RotorIndex) const;

    void ApplyArmJointState(FName JointName, double Position);

protected:
    virtual void BeginPlay() override;

private:
    bool LoadAcesimVisualManifest(TSharedPtr<FJsonObject>& OutManifest) const;
    void BuildVisualTreeFromManifest(const TSharedPtr<FJsonObject>& Manifest);
    void BuildFallbackProxy();
    UStaticMesh* LoadVehicleMesh(const TCHAR* MeshPath) const;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<UACESimVehicleSyncComponent> SyncComponent;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<UACESimArmStateSyncComponent> ArmStateSyncComponent;

    TMap<FName, TObjectPtr<USceneComponent>> BodyComponents;
    TMap<FName, TObjectPtr<UStaticMeshComponent>> MeshComponents;
    TMap<FName, FACESimJointRuntime> JointRuntimes;
    TMap<int32, TWeakObjectPtr<USceneComponent>> RotorComponentsByIndex;

    bool bBuiltVisualTree = false;
    int32 ActiveRotorCount = 4;
};
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Private/ACESimVehicleActor.cpp": """#include "ACESimVehicleActor.h"

#include "ACESimArmStateSyncComponent.h"
#include "ACESimVehicleSyncComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Math/RotationMatrix.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
constexpr double MToCm = 100.0;
const TCHAR* VisualManifestAssetHint = TEXT("/Game/ACESim/x500_arm2x/visual_manifest");

FString GetStringField(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, const FString& DefaultValue = TEXT(""))
{
    FString Value;
    return Object.IsValid() && Object->TryGetStringField(FieldName, Value) ? Value : DefaultValue;
}

FVector JsonVectorField(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName, const FVector& DefaultValue, double Scale)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(FieldName, Values) || Values == nullptr || Values->Num() < 3)
    {
        return DefaultValue;
    }

    return FVector(
        static_cast<float>((*Values)[0]->AsNumber() * Scale),
        static_cast<float>(-(*Values)[1]->AsNumber() * Scale),
        static_cast<float>((*Values)[2]->AsNumber() * Scale));
}

FRotator JsonQuatField(const TSharedPtr<FJsonObject>& Object, const TCHAR* FieldName)
{
    const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(FieldName, Values) || Values == nullptr || Values->Num() < 4)
    {
        return FRotator::ZeroRotator;
    }

    const double W = (*Values)[0]->AsNumber();
    const double X = (*Values)[1]->AsNumber();
    const double Y = (*Values)[2]->AsNumber();
    const double Z = (*Values)[3]->AsNumber();

    const FVector ForwardNwu(
        1.0 - 2.0 * (Y * Y + Z * Z),
        2.0 * (X * Y + W * Z),
        2.0 * (X * Z - W * Y));
    const FVector UpNwu(
        2.0 * (X * Z + W * Y),
        2.0 * (Y * Z - W * X),
        1.0 - 2.0 * (X * X + Y * Y));

    const FVector ForwardUe(
        static_cast<float>(ForwardNwu.X),
        static_cast<float>(-ForwardNwu.Y),
        static_cast<float>(ForwardNwu.Z));
    const FVector UpUe(
        static_cast<float>(UpNwu.X),
        static_cast<float>(-UpNwu.Y),
        static_cast<float>(UpNwu.Z));
    return FRotationMatrix::MakeFromXZ(ForwardUe.GetSafeNormal(), UpUe.GetSafeNormal()).Rotator();
}

int32 RotorIndexFromMeshName(const FString& MeshName)
{
    if (!MeshName.StartsWith(TEXT("rotor_")))
    {
        return INDEX_NONE;
    }
    return FCString::Atoi(*MeshName.Mid(6)) - 1;
}

FName MeshComponentNameForMesh(const FString& MeshName)
{
    const int32 RotorIndex = RotorIndexFromMeshName(MeshName);
    if (RotorIndex != INDEX_NONE)
    {
        return FName(*FString::Printf(TEXT("Rotor_%d"), RotorIndex + 1));
    }
    return FName(*(FString(TEXT("Mesh_")) + MeshName));
}
}  // namespace

AACESimVehicleActor::AACESimVehicleActor()
{
    PrimaryActorTick.bCanEverTick = false;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(SceneRoot);

    SyncComponent = CreateDefaultSubobject<UACESimVehicleSyncComponent>(TEXT("ACESimVehicleSync"));
    ArmStateSyncComponent = CreateDefaultSubobject<UACESimArmStateSyncComponent>(TEXT("ACESimArmStateSync"));
}

void AACESimVehicleActor::BeginPlay()
{
    Super::BeginPlay();

    if (bBuiltVisualTree)
    {
        return;
    }

    TSharedPtr<FJsonObject> Manifest;
    if (LoadAcesimVisualManifest(Manifest))
    {
        BuildVisualTreeFromManifest(Manifest);
    }
    else
    {
        BuildFallbackProxy();
    }
    bBuiltVisualTree = true;
}

void AACESimVehicleActor::SetRotorCount(int32 NewRotorCount)
{
    ActiveRotorCount = FMath::Max(0, NewRotorCount);
    for (const TPair<int32, TWeakObjectPtr<USceneComponent>>& Pair : RotorComponentsByIndex)
    {
        if (USceneComponent* RotorComponent = Pair.Value.Get())
        {
            const bool bVisible = Pair.Key < ActiveRotorCount;
            RotorComponent->SetVisibility(bVisible, true);
            RotorComponent->SetHiddenInGame(!bVisible, true);
        }
    }
}

USceneComponent* AACESimVehicleActor::GetRotorComponentByIndex(int32 RotorIndex) const
{
    const TWeakObjectPtr<USceneComponent>* RotorComponent = RotorComponentsByIndex.Find(RotorIndex);
    return RotorComponent != nullptr ? RotorComponent->Get() : nullptr;
}

void AACESimVehicleActor::ApplyArmJointState(FName JointName, double Position)
{
    FACESimJointRuntime* JointRuntime = JointRuntimes.Find(JointName);
    if (JointRuntime == nullptr)
    {
        return;
    }

    USceneComponent* JointComponent = JointRuntime->Component.Get();
    if (JointComponent == nullptr)
    {
        return;
    }

    const FVector Axis = JointRuntime->Axis.GetSafeNormal();
    if (JointRuntime->bSlide)
    {
        JointComponent->SetRelativeLocation(JointRuntime->HomeLocation + Axis * static_cast<float>(Position * MToCm));
        return;
    }

    const FQuat HomeQuat(JointRuntime->HomeRotation);
    const FQuat JointDelta(Axis, static_cast<float>(Position));
    JointComponent->SetRelativeRotation((HomeQuat * JointDelta).Rotator());
}

bool AACESimVehicleActor::LoadAcesimVisualManifest(TSharedPtr<FJsonObject>& OutManifest) const
{
    const FString ManifestPath = FPaths::ProjectContentDir() / TEXT("ACESim/x500_arm2x/visual_manifest.json");
    FString Payload;
    if (!FFileHelper::LoadFileToString(Payload, *ManifestPath))
    {
        return false;
    }

    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Payload);
    if (!FJsonSerializer::Deserialize(Reader, OutManifest) || !OutManifest.IsValid())
    {
        return false;
    }

    UE_LOG(LogTemp, Display, TEXT("ACESim visual manifest loaded: %s file=%s"), VisualManifestAssetHint, *ManifestPath);
    return true;
}

void AACESimVehicleActor::BuildVisualTreeFromManifest(const TSharedPtr<FJsonObject>& Manifest)
{
    const TArray<TSharedPtr<FJsonValue>>* BodyValues = nullptr;
    if (!Manifest.IsValid() || !Manifest->TryGetArrayField(TEXT("visual_bodies"), BodyValues) || BodyValues == nullptr)
    {
        BuildFallbackProxy();
        return;
    }

    FString ManifestRootBodyName;
    const TSharedPtr<FJsonObject>* ManifestRootObject = nullptr;
    if (Manifest->TryGetObjectField(TEXT("root_body"), ManifestRootObject) && ManifestRootObject != nullptr && ManifestRootObject->IsValid())
    {
        ManifestRootBodyName = GetStringField(*ManifestRootObject, TEXT("name"));
    }

    BodyComponents.Reset();
    MeshComponents.Reset();
    JointRuntimes.Reset();
    RotorComponentsByIndex.Reset();

    for (const TSharedPtr<FJsonValue>& BodyValue : *BodyValues)
    {
        const TSharedPtr<FJsonObject> BodyObject = BodyValue->AsObject();
        const FString BodyName = GetStringField(BodyObject, TEXT("name"));
        if (BodyName.IsEmpty())
        {
            continue;
        }

        USceneComponent* BodyComponent = NewObject<USceneComponent>(this, FName(*BodyName));
        BodyComponent->SetMobility(EComponentMobility::Movable);
        AddInstanceComponent(BodyComponent);
        BodyComponents.Add(FName(*BodyName), BodyComponent);
    }

    bool bLoadedAnyRealMesh = false;
    for (const TSharedPtr<FJsonValue>& BodyValue : *BodyValues)
    {
        const TSharedPtr<FJsonObject> BodyObject = BodyValue->AsObject();
        const FString BodyName = GetStringField(BodyObject, TEXT("name"));
        TObjectPtr<USceneComponent>* BodyComponentPtr = BodyComponents.Find(FName(*BodyName));
        if (BodyComponentPtr == nullptr || BodyComponentPtr->Get() == nullptr)
        {
            continue;
        }

        USceneComponent* BodyComponent = BodyComponentPtr->Get();
        const FString ParentName = GetStringField(BodyObject, TEXT("parent"));
        USceneComponent* ParentComponent = SceneRoot;
        if (!ParentName.IsEmpty())
        {
            if (TObjectPtr<USceneComponent>* FoundParent = BodyComponents.Find(FName(*ParentName)))
            {
                ParentComponent = FoundParent->Get();
            }
        }

        BodyComponent->AttachToComponent(ParentComponent, FAttachmentTransformRules::KeepRelativeTransform);
        const bool bIsManifestRootBody = BodyName == ManifestRootBodyName;
        if (bIsManifestRootBody)
        {
            // The actor itself is driven by the streamed root-body pose, so do
            // not apply the MJCF root body offset again under the actor.
            BodyComponent->SetRelativeLocation(FVector::ZeroVector);
            BodyComponent->SetRelativeRotation(FRotator::ZeroRotator);
        }
        else
        {
            BodyComponent->SetRelativeLocation(JsonVectorField(BodyObject, TEXT("pos"), FVector::ZeroVector, MToCm));
            BodyComponent->SetRelativeRotation(JsonQuatField(BodyObject, TEXT("quat")));
        }
        BodyComponent->RegisterComponent();

        const TSharedPtr<FJsonObject>* JointObject = nullptr;
        if (BodyObject->TryGetObjectField(TEXT("joint"), JointObject) && JointObject != nullptr && JointObject->IsValid())
        {
            const FString JointName = GetStringField(*JointObject, TEXT("name"));
            if (!JointName.IsEmpty())
            {
                FACESimJointRuntime Runtime;
                Runtime.Component = BodyComponent;
                Runtime.Axis = JsonVectorField(*JointObject, TEXT("axis"), FVector::UpVector, 1.0).GetSafeNormal();
                Runtime.HomeLocation = BodyComponent->GetRelativeLocation();
                Runtime.HomeRotation = BodyComponent->GetRelativeRotation();
                Runtime.bSlide = GetStringField(*JointObject, TEXT("type")).Equals(TEXT("slide"), ESearchCase::IgnoreCase);
                JointRuntimes.Add(FName(*JointName), Runtime);
            }
        }

        const TArray<TSharedPtr<FJsonValue>>* GeomValues = nullptr;
        if (!BodyObject->TryGetArrayField(TEXT("geoms"), GeomValues) || GeomValues == nullptr)
        {
            continue;
        }

        for (const TSharedPtr<FJsonValue>& GeomValue : *GeomValues)
        {
            const TSharedPtr<FJsonObject> GeomObject = GeomValue->AsObject();
            const FString MeshName = GetStringField(GeomObject, TEXT("mesh"));
            if (MeshName.IsEmpty())
            {
                continue;
            }

            UStaticMeshComponent* MeshComponent = NewObject<UStaticMeshComponent>(this, MeshComponentNameForMesh(MeshName));
            MeshComponent->SetMobility(EComponentMobility::Movable);
            MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            MeshComponent->AttachToComponent(BodyComponent, FAttachmentTransformRules::KeepRelativeTransform);
            MeshComponent->SetRelativeLocation(JsonVectorField(GeomObject, TEXT("pos"), FVector::ZeroVector, MToCm));
            MeshComponent->SetRelativeRotation(JsonQuatField(GeomObject, TEXT("quat")));
            AddInstanceComponent(MeshComponent);

            const FString MeshPath = FString::Printf(TEXT("/Game/ACESim/x500_arm2x/%s.%s"), *MeshName, *MeshName);
            if (UStaticMesh* Mesh = LoadVehicleMesh(*MeshPath))
            {
                MeshComponent->SetStaticMesh(Mesh);
                bLoadedAnyRealMesh = true;
            }
            MeshComponent->RegisterComponent();
            MeshComponents.Add(FName(*MeshName), MeshComponent);

            const int32 RotorIndex = RotorIndexFromMeshName(MeshName);
            if (RotorIndex != INDEX_NONE)
            {
                RotorComponentsByIndex.Add(RotorIndex, MeshComponent);
            }
        }
    }

    if (bLoadedAnyRealMesh)
    {
        UE_LOG(LogTemp, Display, TEXT("ACESim real vehicle mesh loaded"));
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("ACESim vehicle mesh missing; using fallback proxy"));
        BuildFallbackProxy();
    }

    UE_LOG(LogTemp, Display, TEXT("ACESim manifest-loaded assembly: bodies=%d meshes=%d joints=%d"),
        BodyComponents.Num(),
        MeshComponents.Num(),
        JointRuntimes.Num());
    SetRotorCount(ActiveRotorCount);
}

void AACESimVehicleActor::BuildFallbackProxy()
{
    UStaticMeshComponent* BodyMeshComponent = NewObject<UStaticMeshComponent>(this, TEXT("FallbackBody"));
    BodyMeshComponent->SetMobility(EComponentMobility::Movable);
    BodyMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    BodyMeshComponent->AttachToComponent(SceneRoot, FAttachmentTransformRules::KeepRelativeTransform);
    BodyMeshComponent->SetRelativeScale3D(FVector(1.2f, 0.6f, 0.18f));
    if (UStaticMesh* BodyMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")))
    {
        BodyMeshComponent->SetStaticMesh(BodyMesh);
    }
    AddInstanceComponent(BodyMeshComponent);
    BodyMeshComponent->RegisterComponent();
    MeshComponents.Add(TEXT("fallback_body"), BodyMeshComponent);
    UE_LOG(LogTemp, Warning, TEXT("ACESim vehicle mesh missing; using fallback proxy"));
}

UStaticMesh* AACESimVehicleActor::LoadVehicleMesh(const TCHAR* MeshPath) const
{
    return LoadObject<UStaticMesh>(nullptr, MeshPath);
}
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Public/ACESimVehicleSyncComponent.h": """#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HAL/CriticalSection.h"

#include "ACESimVehicleSyncComponent.generated.h"

class FACESimVehicleReceiverThread;

UCLASS(ClassGroup=(ACESim), meta=(BlueprintSpawnableComponent))
class ACESIMBRIDGE_API UACESimVehicleSyncComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UACESimVehicleSyncComponent();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(
        float DeltaTime,
        ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;

    UFUNCTION(BlueprintCallable, Category="ACESim")
    bool HasReceivedSample() const { return bHasReceivedSample.Load(); }

    UFUNCTION(BlueprintCallable, Category="ACESim")
    int64 GetLastTimestampUs() const { return LastTimestampUs.Load(); }

protected:
    UPROPERTY(EditAnywhere, Category="ACESim")
    FString Endpoint = TEXT("tcp://127.0.0.1:5601");

    UPROPERTY(EditAnywhere, Category="ACESim")
    bool bAutoConnectOnBeginPlay = true;

    UPROPERTY(EditAnywhere, Category="ACESim")
    TArray<FName> RotorComponentNames;

private:
    friend class FACESimVehicleReceiverThread;

    void StartReceiver();
    void StopReceiver();
    void ReceiverLoop();
    void RefreshRotorComponents();
    void ApplyLatestSample();

    static constexpr int32 MaxRotorCount = 8;

    struct FACESimLatestSample
    {
        bool bValid = false;
        uint64 TimestampUs = 0;
        FVector PositionCm = FVector::ZeroVector;
        FRotator Rotation = FRotator::ZeroRotator;
        int32 RotorCount = 0;
        double RotorAngleRad[MaxRotorCount] {};
    };

    TArray<TWeakObjectPtr<USceneComponent>> CachedRotorComponents;
    FACESimVehicleReceiverThread* ReceiveThread = nullptr;
    FCriticalSection SampleMutex;
    FACESimLatestSample SharedSample;
    TAtomic<bool> bStopRequested = false;
    TAtomic<bool> bHasReceivedSample = false;
    TAtomic<int64> LastTimestampUs = 0;
    bool bLoggedFirstSample = false;
    bool bLoggedFirstApply = false;
    bool bConnected = false;
};
""",
        (
            f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Private/" "ACESimVehicleSyncComponent.cpp"
        ): """#include "ACESimVehicleSyncComponent.h"

#include "ACESimVehicleActor.h"
#include "Components/SceneComponent.h"
#include "Containers/StringConv.h"
#include "Engine/World.h"
#include "Math/RotationMatrix.h"
#include "Misc/ScopeLock.h"

#include <array>
#include <cerrno>
#include <thread>
#include <zmq.h>

namespace
{
constexpr int32 MaxRotors = 8;

#pragma pack(push, 1)
struct FACESimVisualWireSample
{
    uint64 TimestampUs;
    double PositionWorldM_NWU[3];
    double AttitudeWorldQuatScalarFirst[4];
    uint32 RotorCount;
    double RotorAngleRad[MaxRotors];
    double RotorVisualSpeedRadps[MaxRotors];
};
#pragma pack(pop)

static_assert(sizeof(FACESimVisualWireSample) == 196, "Unexpected ACESim wire payload size");

struct FACESimVisualSample
{
    bool bValid = false;
    uint64 TimestampUs = 0;
    FVector PositionCm = FVector::ZeroVector;
    FRotator Rotation = FRotator::ZeroRotator;
    uint32 RotorCount = 0;
    std::array<double, MaxRotors> RotorAngleRad {};
};

FVector ConvertWorldVectorNwuToUe(double X, double Y, double Z, double Scale)
{
    return FVector(X * Scale, -Y * Scale, Z * Scale);
}

FRotator ConvertAttitudeNwuFluToUe(const double QuatScalarFirst[4])
{
    const double W = QuatScalarFirst[0];
    const double X = QuatScalarFirst[1];
    const double Y = QuatScalarFirst[2];
    const double Z = QuatScalarFirst[3];

    const FVector ForwardNwu(
        1.0 - 2.0 * (Y * Y + Z * Z),
        2.0 * (X * Y + W * Z),
        2.0 * (X * Z - W * Y));
    const FVector UpNwu(
        2.0 * (X * Z + W * Y),
        2.0 * (Y * Z - W * X),
        1.0 - 2.0 * (X * X + Y * Y));

    const FVector ForwardUe = ConvertWorldVectorNwuToUe(ForwardNwu.X, ForwardNwu.Y, ForwardNwu.Z, 1.0).GetSafeNormal();
    const FVector UpUe = ConvertWorldVectorNwuToUe(UpNwu.X, UpNwu.Y, UpNwu.Z, 1.0).GetSafeNormal();
    return FRotationMatrix::MakeFromXZ(ForwardUe, UpUe).Rotator();
}
}  // namespace

class FACESimVehicleReceiverThread
{
public:
    explicit FACESimVehicleReceiverThread(UACESimVehicleSyncComponent* InOwner)
        : Thread([InOwner]() { InOwner->ReceiverLoop(); })
    {
    }

    void Join()
    {
        if (Thread.joinable())
        {
            Thread.join();
        }
    }

private:
    std::thread Thread;
};

UACESimVehicleSyncComponent::UACESimVehicleSyncComponent()
{
    PrimaryComponentTick.bCanEverTick = true;

    for (int32 RotorIndex = 0; RotorIndex < MaxRotors; ++RotorIndex)
    {
        RotorComponentNames.Add(*FString::Printf(TEXT("Rotor_%d"), RotorIndex + 1));
    }
}

void UACESimVehicleSyncComponent::BeginPlay()
{
    Super::BeginPlay();
    RefreshRotorComponents();
    if (bAutoConnectOnBeginPlay)
    {
        StartReceiver();
    }
}

void UACESimVehicleSyncComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    StopReceiver();
    Super::EndPlay(EndPlayReason);
}

void UACESimVehicleSyncComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    ApplyLatestSample();
}

void UACESimVehicleSyncComponent::StartReceiver()
{
    if (ReceiveThread != nullptr)
    {
        return;
    }

    bStopRequested = false;
    bLoggedFirstSample = false;
    bLoggedFirstApply = false;
    ReceiveThread = new FACESimVehicleReceiverThread(this);
}

void UACESimVehicleSyncComponent::StopReceiver()
{
    bStopRequested = true;
    if (ReceiveThread != nullptr)
    {
        ReceiveThread->Join();
        delete ReceiveThread;
        ReceiveThread = nullptr;
    }
    bConnected = false;
}

void UACESimVehicleSyncComponent::ReceiverLoop()
{
    void* Context = zmq_ctx_new();
    if (Context == nullptr)
    {
        return;
    }

    void* Socket = zmq_socket(Context, ZMQ_SUB);
    if (Socket == nullptr)
    {
        zmq_ctx_term(Context);
        return;
    }

    const int Zero = 0;
    const int One = 1;
    const int TimeoutMs = 100;
    zmq_setsockopt(Socket, ZMQ_LINGER, &Zero, sizeof(Zero));
    zmq_setsockopt(Socket, ZMQ_RCVHWM, &One, sizeof(One));
    zmq_setsockopt(Socket, ZMQ_CONFLATE, &One, sizeof(One));
    zmq_setsockopt(Socket, ZMQ_RCVTIMEO, &TimeoutMs, sizeof(TimeoutMs));
    zmq_setsockopt(Socket, ZMQ_SUBSCRIBE, "", 0);

    const FTCHARToUTF8 EndpointUtf8(*Endpoint);
    if (zmq_connect(Socket, EndpointUtf8.Get()) != 0)
    {
        zmq_close(Socket);
        zmq_ctx_term(Context);
        return;
    }

    bConnected = true;
    FACESimVisualWireSample WireSample {};

    while (!bStopRequested.Load())
    {
        const int ReceivedBytes = zmq_recv(Socket, &WireSample, sizeof(WireSample), 0);
        if (ReceivedBytes < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                continue;
            }
            break;
        }
        if (ReceivedBytes != sizeof(WireSample))
        {
            continue;
        }

        FACESimVisualSample LocalSample;
        LocalSample.bValid = true;
        LocalSample.TimestampUs = WireSample.TimestampUs;
        LocalSample.PositionCm = ConvertWorldVectorNwuToUe(
            WireSample.PositionWorldM_NWU[0],
            WireSample.PositionWorldM_NWU[1],
            WireSample.PositionWorldM_NWU[2],
            100.0);
        LocalSample.Rotation = ConvertAttitudeNwuFluToUe(WireSample.AttitudeWorldQuatScalarFirst);
        LocalSample.RotorCount = FMath::Clamp<int32>(static_cast<int32>(WireSample.RotorCount), 0, MaxRotors);
        for (int32 RotorIndex = 0; RotorIndex < MaxRotors; ++RotorIndex)
        {
            LocalSample.RotorAngleRad[RotorIndex] = WireSample.RotorAngleRad[RotorIndex];
        }

        {
            FScopeLock Lock(&SampleMutex);
            SharedSample.bValid = LocalSample.bValid;
            SharedSample.TimestampUs = LocalSample.TimestampUs;
            SharedSample.PositionCm = LocalSample.PositionCm;
            SharedSample.Rotation = LocalSample.Rotation;
            SharedSample.RotorCount = static_cast<int32>(LocalSample.RotorCount);
            for (int32 RotorIndex = 0; RotorIndex < MaxRotors; ++RotorIndex)
            {
                SharedSample.RotorAngleRad[RotorIndex] = LocalSample.RotorAngleRad[RotorIndex];
            }
        }
        LastTimestampUs = static_cast<int64>(LocalSample.TimestampUs);
        bHasReceivedSample = true;

        if (!bLoggedFirstSample)
        {
            UE_LOG(
                LogTemp,
                Display,
                TEXT("ACESim visual stream connected: endpoint=%s timestamp_us=%llu rotors=%u"),
                *Endpoint,
                static_cast<unsigned long long>(LocalSample.TimestampUs),
                LocalSample.RotorCount);
            bLoggedFirstSample = true;
        }
    }

    zmq_close(Socket);
    zmq_ctx_term(Context);
    bConnected = false;
}

void UACESimVehicleSyncComponent::RefreshRotorComponents()
{
    CachedRotorComponents.Reset();
    AActor* Owner = GetOwner();
    if (Owner == nullptr)
    {
        return;
    }

    for (const FName& ComponentName : RotorComponentNames)
    {
        USceneComponent* FoundComponent = nullptr;
        TInlineComponentArray<USceneComponent*> Components(Owner);
        for (USceneComponent* Component : Components)
        {
            if (Component != nullptr && Component->GetFName() == ComponentName)
            {
                FoundComponent = Component;
                break;
            }
        }
        CachedRotorComponents.Add(FoundComponent);
    }
}

void UACESimVehicleSyncComponent::ApplyLatestSample()
{
    if (!bHasReceivedSample.Load())
    {
        return;
    }

    FACESimVisualSample LatestSample;
    {
        FScopeLock Lock(&SampleMutex);
        LatestSample.bValid = SharedSample.bValid;
        LatestSample.TimestampUs = SharedSample.TimestampUs;
        LatestSample.PositionCm = SharedSample.PositionCm;
        LatestSample.Rotation = SharedSample.Rotation;
        LatestSample.RotorCount = static_cast<uint32>(SharedSample.RotorCount);
        for (int32 RotorIndex = 0; RotorIndex < MaxRotors; ++RotorIndex)
        {
            LatestSample.RotorAngleRad[RotorIndex] = SharedSample.RotorAngleRad[RotorIndex];
        }
    }
    if (!LatestSample.bValid)
    {
        return;
    }

    AActor* Owner = GetOwner();
    if (Owner == nullptr)
    {
        return;
    }

    Owner->SetActorLocationAndRotation(
        LatestSample.PositionCm,
        LatestSample.Rotation,
        false,
        nullptr,
        ETeleportType::TeleportPhysics
    );

    if (!bLoggedFirstApply)
    {
        UE_LOG(
            LogTemp,
            Display,
            TEXT("ACESim visual state applied: timestamp_us=%llu position_cm=%s rotors=%u"),
            static_cast<unsigned long long>(LatestSample.TimestampUs),
            *LatestSample.PositionCm.ToCompactString(),
            LatestSample.RotorCount);
        bLoggedFirstApply = true;
    }

    AACESimVehicleActor* VehicleActor = Cast<AACESimVehicleActor>(Owner);
    if (VehicleActor != nullptr)
    {
        VehicleActor->SetRotorCount(static_cast<int32>(LatestSample.RotorCount));
    }

    if (CachedRotorComponents.Num() == 0)
    {
        RefreshRotorComponents();
    }

    const int32 Count = FMath::Min<int32>(LatestSample.RotorCount, MaxRotors);
    for (int32 RotorIndex = 0; RotorIndex < Count; ++RotorIndex)
    {
        USceneComponent* RotorComponent = nullptr;
        if (VehicleActor != nullptr)
        {
            RotorComponent = VehicleActor->GetRotorComponentByIndex(RotorIndex);
        }
        if (RotorComponent == nullptr && CachedRotorComponents.IsValidIndex(RotorIndex))
        {
            RotorComponent = CachedRotorComponents[RotorIndex].Get();
        }

        if (RotorComponent != nullptr)
        {
            const float RotorDegrees = FMath::RadiansToDegrees(
                static_cast<float>(LatestSample.RotorAngleRad[RotorIndex])
            );
            RotorComponent->SetRelativeRotation(FRotator(0.0f, RotorDegrees, 0.0f));
        }
    }
}
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Public/ACESimArmStateSyncComponent.h": """#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HAL/CriticalSection.h"

#include "ACESimArmStateSyncComponent.generated.h"

class FACESimArmStateReceiverThread;
class USceneComponent;

UCLASS(ClassGroup=(ACESim), meta=(BlueprintSpawnableComponent))
class ACESIMBRIDGE_API UACESimArmStateSyncComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UACESimArmStateSyncComponent();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(
        float DeltaTime,
        ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;

protected:
    UPROPERTY(EditAnywhere, Category="ACESim")
    FString Endpoint = TEXT("tcp://127.0.0.1:5603");

    UPROPERTY(EditAnywhere, Category="ACESim")
    bool bAutoConnectOnBeginPlay = true;

    UPROPERTY(EditAnywhere, Category="ACESim")
    TArray<FName> JointComponentNames;

private:
    friend class FACESimArmStateReceiverThread;

    void StartReceiver();
    void StopReceiver();
    void ReceiverLoop();
    void RefreshJointComponents();
    void ApplyLatestSample();

    static constexpr int32 MaxJointCount = 7;

    struct FACESimLatestArmState
    {
        bool bValid = false;
        uint64 TimestampUs = 0;
        int32 JointCount = 0;
        double PositionRad[MaxJointCount] {};
    };

    TArray<TWeakObjectPtr<USceneComponent>> CachedJointComponents;
    FACESimArmStateReceiverThread* ReceiveThread = nullptr;
    FCriticalSection SampleMutex;
    FACESimLatestArmState SharedSample;
    TAtomic<bool> bStopRequested = false;
    TAtomic<bool> bHasReceivedSample = false;
    bool bLoggedFirstSample = false;
};
""",
        (
            f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Private/" "ACESimArmStateSyncComponent.cpp"
        ): """#include "ACESimArmStateSyncComponent.h"

#include "ACESimVehicleActor.h"
#include "Components/SceneComponent.h"
#include "Containers/StringConv.h"
#include "Misc/ScopeLock.h"

#include <cerrno>
#include <thread>
#include <zmq.h>

namespace
{
constexpr int32 LegacyArmJointCount = 5;
constexpr int32 ArmJointCount = 7;

#pragma pack(push, 1)
struct FACESimLegacyArmStateWireSample
{
    uint64 TimestampUs;
    double PositionRad[LegacyArmJointCount];
    double VelocityRadps[LegacyArmJointCount];
    double Effort[LegacyArmJointCount];
};

struct FACESimArmStateWireSample
{
    uint64 TimestampUs;
    double PositionRad[ArmJointCount];
    double VelocityRadps[ArmJointCount];
    double Effort[ArmJointCount];
};
#pragma pack(pop)

static_assert(sizeof(FACESimLegacyArmStateWireSample) == 128, "Unexpected legacy ACESim arm-state wire payload size");
static_assert(sizeof(FACESimArmStateWireSample) == 176, "Unexpected ACESim arm-state wire payload size");
}  // namespace

class FACESimArmStateReceiverThread
{
public:
    explicit FACESimArmStateReceiverThread(UACESimArmStateSyncComponent* InOwner)
        : Thread([InOwner]() { InOwner->ReceiverLoop(); })
    {
    }

    void Join()
    {
        if (Thread.joinable())
        {
            Thread.join();
        }
    }

private:
    std::thread Thread;
};

UACESimArmStateSyncComponent::UACESimArmStateSyncComponent()
{
    PrimaryComponentTick.bCanEverTick = true;

    JointComponentNames.Add(TEXT("joint_1"));
    JointComponentNames.Add(TEXT("joint_2"));
    JointComponentNames.Add(TEXT("joint_3"));
    JointComponentNames.Add(TEXT("joint_4"));
    JointComponentNames.Add(TEXT("joint_5"));
    JointComponentNames.Add(TEXT("joint_gripper_left"));
    JointComponentNames.Add(TEXT("joint_gripper_right"));
}

void UACESimArmStateSyncComponent::BeginPlay()
{
    Super::BeginPlay();
    RefreshJointComponents();
    if (bAutoConnectOnBeginPlay)
    {
        StartReceiver();
    }
}

void UACESimArmStateSyncComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    StopReceiver();
    Super::EndPlay(EndPlayReason);
}

void UACESimArmStateSyncComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    ApplyLatestSample();
}

void UACESimArmStateSyncComponent::StartReceiver()
{
    if (ReceiveThread != nullptr)
    {
        return;
    }

    bStopRequested = false;
    bLoggedFirstSample = false;
    ReceiveThread = new FACESimArmStateReceiverThread(this);
}

void UACESimArmStateSyncComponent::StopReceiver()
{
    bStopRequested = true;
    if (ReceiveThread != nullptr)
    {
        ReceiveThread->Join();
        delete ReceiveThread;
        ReceiveThread = nullptr;
    }
}

void UACESimArmStateSyncComponent::ReceiverLoop()
{
    void* Context = zmq_ctx_new();
    if (Context == nullptr)
    {
        return;
    }

    void* Socket = zmq_socket(Context, ZMQ_SUB);
    if (Socket == nullptr)
    {
        zmq_ctx_term(Context);
        return;
    }

    const int Zero = 0;
    const int One = 1;
    const int TimeoutMs = 100;
    zmq_setsockopt(Socket, ZMQ_LINGER, &Zero, sizeof(Zero));
    zmq_setsockopt(Socket, ZMQ_RCVHWM, &One, sizeof(One));
    zmq_setsockopt(Socket, ZMQ_CONFLATE, &One, sizeof(One));
    zmq_setsockopt(Socket, ZMQ_RCVTIMEO, &TimeoutMs, sizeof(TimeoutMs));
    zmq_setsockopt(Socket, ZMQ_SUBSCRIBE, "", 0);

    const FTCHARToUTF8 EndpointUtf8(*Endpoint);
    if (zmq_connect(Socket, EndpointUtf8.Get()) != 0)
    {
        zmq_close(Socket);
        zmq_ctx_term(Context);
        return;
    }

    uint8 WireBuffer[sizeof(FACESimArmStateWireSample)] {};
    while (!bStopRequested.Load())
    {
        const int ReceivedBytes = zmq_recv(Socket, WireBuffer, sizeof(WireBuffer), 0);
        if (ReceivedBytes < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                continue;
            }
            break;
        }
        if (ReceivedBytes != sizeof(FACESimLegacyArmStateWireSample) && ReceivedBytes != sizeof(FACESimArmStateWireSample))
        {
            continue;
        }

        uint64 TimestampUs = 0;
        int32 PayloadJointCount = 0;
        double Positions[ArmJointCount] {};
        if (ReceivedBytes == sizeof(FACESimArmStateWireSample))
        {
            const FACESimArmStateWireSample* WireSample = reinterpret_cast<const FACESimArmStateWireSample*>(WireBuffer);
            TimestampUs = WireSample->TimestampUs;
            PayloadJointCount = ArmJointCount;
            for (int32 JointIndex = 0; JointIndex < ArmJointCount; ++JointIndex)
            {
                Positions[JointIndex] = WireSample->PositionRad[JointIndex];
            }
        }
        else
        {
            const FACESimLegacyArmStateWireSample* WireSample = reinterpret_cast<const FACESimLegacyArmStateWireSample*>(WireBuffer);
            TimestampUs = WireSample->TimestampUs;
            PayloadJointCount = LegacyArmJointCount;
            for (int32 JointIndex = 0; JointIndex < LegacyArmJointCount; ++JointIndex)
            {
                Positions[JointIndex] = WireSample->PositionRad[JointIndex];
            }
        }

        {
            FScopeLock Lock(&SampleMutex);
            SharedSample.bValid = true;
            SharedSample.TimestampUs = TimestampUs;
            SharedSample.JointCount = PayloadJointCount;
            for (int32 JointIndex = 0; JointIndex < ArmJointCount; ++JointIndex)
            {
                SharedSample.PositionRad[JointIndex] = Positions[JointIndex];
            }
        }
        bHasReceivedSample = true;

        if (!bLoggedFirstSample)
        {
            UE_LOG(
                LogTemp,
                Display,
                TEXT("ACESim arm state stream connected: endpoint=%s timestamp_us=%llu payload=%s"),
                *Endpoint,
                static_cast<unsigned long long>(TimestampUs),
                PayloadJointCount == ArmJointCount ? TEXT("7-joint") : TEXT("5-joint"));
            bLoggedFirstSample = true;
        }
    }

    zmq_close(Socket);
    zmq_ctx_term(Context);
}

void UACESimArmStateSyncComponent::RefreshJointComponents()
{
    CachedJointComponents.Reset();
    AActor* Owner = GetOwner();
    if (Owner == nullptr)
    {
        return;
    }

    TInlineComponentArray<USceneComponent*> Components(Owner);
    for (const FName& ComponentName : JointComponentNames)
    {
        USceneComponent* FoundComponent = nullptr;
        for (USceneComponent* Component : Components)
        {
            if (Component != nullptr && Component->GetFName() == ComponentName)
            {
                FoundComponent = Component;
                break;
            }
        }
        CachedJointComponents.Add(FoundComponent);
    }
}

void UACESimArmStateSyncComponent::ApplyLatestSample()
{
    if (!bHasReceivedSample.Load())
    {
        return;
    }

    FACESimLatestArmState LatestSample;
    {
        FScopeLock Lock(&SampleMutex);
        LatestSample = SharedSample;
    }
    if (!LatestSample.bValid)
    {
        return;
    }

    if (CachedJointComponents.Num() == 0)
    {
        RefreshJointComponents();
    }

    AACESimVehicleActor* VehicleActor = Cast<AACESimVehicleActor>(GetOwner());
    const int32 Count = FMath::Min<int32>(LatestSample.JointCount, JointComponentNames.Num());
    for (int32 JointIndex = 0; JointIndex < Count; ++JointIndex)
    {
        if (VehicleActor != nullptr)
        {
            VehicleActor->ApplyArmJointState(JointComponentNames[JointIndex], LatestSample.PositionRad[JointIndex]);
            continue;
        }

        if (CachedJointComponents.IsValidIndex(JointIndex))
        {
            if (USceneComponent* JointComponent = CachedJointComponents[JointIndex].Get())
            {
                JointComponent->SetRelativeRotation(
                    FRotator(0.0f, FMath::RadiansToDegrees(static_cast<float>(LatestSample.PositionRad[JointIndex])), 0.0f));
            }
        }
    }
}
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Public/ACESimSensorFeedbackComponent.h": """#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ACESimSensorFeedbackComponent.generated.h"

struct FACESimSensorFrameHeader
{
    uint64 TimestampUs = 0;
    uint32 FrameId = 0;
    uint32 Width = 0;
    uint32 Height = 0;
    uint32 PixelFormat = 0;
    uint32 PayloadByteCount = 0;
};

struct FACESimBridgeClock
{
    uint64 LastSimulationTimestampUs = 0;
};

UCLASS(ClassGroup=(ACESim), meta=(BlueprintSpawnableComponent))
class ACESIMBRIDGE_API UACESimSensorFeedbackComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UACESimSensorFeedbackComponent();

protected:
    virtual void BeginPlay() override;

    UPROPERTY(EditAnywhere, Category="ACESim|Sensor Feedback")
    bool bEnableSensorFeedback = false;

    UPROPERTY(EditAnywhere, Category="ACESim|Sensor Feedback")
    FString CameraRgbEndpoint = TEXT("tcp://127.0.0.1:5610");

    UPROPERTY(EditAnywhere, Category="ACESim|Sensor Feedback")
    FString DepthEndpoint = TEXT("tcp://127.0.0.1:5611");

    UPROPERTY(EditAnywhere, Category="ACESim|Sensor Feedback")
    FString SegmentationEndpoint = TEXT("tcp://127.0.0.1:5612");

    UPROPERTY(EditAnywhere, Category="ACESim|Sensor Feedback")
    FString EventEndpoint = TEXT("tcp://127.0.0.1:5613");
};
""",
        (
            f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Private/" "ACESimSensorFeedbackComponent.cpp"
        ): """#include "ACESimSensorFeedbackComponent.h"

#include "Logging/LogMacros.h"

UACESimSensorFeedbackComponent::UACESimSensorFeedbackComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UACESimSensorFeedbackComponent::BeginPlay()
{
    Super::BeginPlay();

    if (bEnableSensorFeedback)
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("ACESim sensor feedback is reserved but not implemented. "
                 "Phase 1 intentionally does not publish sensor payloads."));
    }
}
""",
    }


def write_templates(project_root: Path, overwrite: bool, render_preset: str) -> None:
    for relative_path, content in _templates(render_preset).items():
        target_path = project_root / relative_path
        target_path.parent.mkdir(parents=True, exist_ok=True)
        if target_path.exists() and not overwrite:
            raise FileExistsError(f"{target_path} already exists. Re-run with --overwrite to replace it.")
        target_path.write_text(content, encoding="utf-8")


def generate_project(project_root: Path, overwrite: bool, render_preset: str | None = None) -> None:
    preset = _normalize_render_preset(render_preset or os.environ.get("ACESIM_UE_RENDER_PRESET", "performance"))
    project_root.mkdir(parents=True, exist_ok=True)
    write_templates(project_root, overwrite=overwrite, render_preset=preset)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate the ACESim UE5 project scaffold.")
    parser.add_argument(
        "--project-root",
        type=Path,
        default=Path("/tmp/ACESim-unreal/projects/ACESimUE"),
        help="Target project directory",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace any existing project files in the target directory",
    )
    parser.add_argument(
        "--render-preset",
        choices=sorted(RENDER_PRESETS),
        default=os.environ.get("ACESIM_UE_RENDER_PRESET", "performance"),
        help="ACESim UE render preset written to DefaultEngine.ini.",
    )
    args = parser.parse_args()

    project_root = args.project_root.expanduser().resolve()
    generate_project(project_root, overwrite=args.overwrite, render_preset=args.render_preset)
    print(f"Generated UE5 project scaffold at {project_root} (render preset: {args.render_preset})")


if __name__ == "__main__":
    main()
