#!/usr/bin/env python3
"""Generate a minimal Unreal Engine 5 project for ACESim visual sync."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

PROJECT_NAME = "ACESimUE"
PLUGIN_NAME = "ACESimBridge"


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


def _templates() -> dict[str, str]:
    return {
        f"{PROJECT_NAME}.uproject": _uproject(),
        "Config/DefaultEngine.ini": """[/Script/Engine.Engine]
+ActiveGameNameRedirects=(OldGameName="TP_Blank",NewGameName="/Script/ACESimUE")

[/Script/EngineSettings.GameMapsSettings]
EditorStartupMap=/Engine/Maps/Templates/OpenWorld
GameDefaultMap=/Engine/Maps/Templates/OpenWorld
""",
        "Config/DefaultGame.ini": """[/Script/EngineSettings.GeneralProjectSettings]
ProjectID=0A9A19D34F8E4D53A0CFF6D3A6A2CB2A
ProjectName=ACESimUE
ProjectVersion=0.1.0
Description=Minimal UE5 project for ACESim MuJoCo visual sync.
""",
        f"Source/{PROJECT_NAME}.Target.cs": """using UnrealBuildTool;
using System.Collections.Generic;

public class ACESimUETarget : TargetRules
{
    public ACESimUETarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_4;
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
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_4;
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
            "InputCore"
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
            "Engine"
        });

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

class USceneComponent;

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

protected:
    virtual void OnConstruction(const FTransform& Transform) override;

private:
    void LayoutRotors();

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, Category="ACESim")
    TArray<TObjectPtr<USceneComponent>> RotorComponents;

    UPROPERTY(EditAnywhere, Category="ACESim")
    int32 RotorCount = 4;

    UPROPERTY(EditAnywhere, Category="ACESim")
    float RotorCircleRadiusCm = 35.0f;
};
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Private/ACESimVehicleActor.cpp": """#include "ACESimVehicleActor.h"

#include "Components/SceneComponent.h"

AACESimVehicleActor::AACESimVehicleActor()
{
    PrimaryActorTick.bCanEverTick = false;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(SceneRoot);

    RotorComponents.Reserve(8);
    for (int32 RotorIndex = 0; RotorIndex < 8; ++RotorIndex)
    {
        const FString Name = FString::Printf(TEXT("Rotor_%d"), RotorIndex + 1);
        USceneComponent* Rotor = CreateDefaultSubobject<USceneComponent>(*Name);
        Rotor->SetupAttachment(SceneRoot);
        RotorComponents.Add(Rotor);
    }
}

void AACESimVehicleActor::SetRotorCount(int32 NewRotorCount)
{
    RotorCount = FMath::Clamp(NewRotorCount, 0, RotorComponents.Num());
    LayoutRotors();
}

USceneComponent* AACESimVehicleActor::GetRotorComponentByIndex(int32 RotorIndex) const
{
    if (!RotorComponents.IsValidIndex(RotorIndex))
    {
        return nullptr;
    }
    return RotorComponents[RotorIndex];
}

void AACESimVehicleActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    LayoutRotors();
}

void AACESimVehicleActor::LayoutRotors()
{
    const int32 ActiveCount = FMath::Clamp(RotorCount, 0, RotorComponents.Num());
    const float StepRadians = ActiveCount > 0 ? (2.0f * PI) / static_cast<float>(ActiveCount) : 0.0f;

    for (int32 RotorIndex = 0; RotorIndex < RotorComponents.Num(); ++RotorIndex)
    {
        USceneComponent* Rotor = RotorComponents[RotorIndex];
        if (Rotor == nullptr)
        {
            continue;
        }

        const bool bVisible = RotorIndex < ActiveCount;
        Rotor->SetVisibility(bVisible, true);
        Rotor->SetHiddenInGame(!bVisible, true);

        if (!bVisible)
        {
            continue;
        }

        const float Angle = StepRadians * RotorIndex;
        const FVector Position(
            RotorCircleRadiusCm * FMath::Cos(Angle),
            RotorCircleRadiusCm * FMath::Sin(Angle),
            0.0f);
        Rotor->SetRelativeLocation(Position);
        Rotor->SetRelativeRotation(FRotator::ZeroRotator);
    }
}
""",
        f"Plugins/{PLUGIN_NAME}/Source/{PLUGIN_NAME}/Public/ACESimVehicleSyncComponent.h": """#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HAL/CriticalSection.h"
#include "ACESimVehicleSyncComponent.generated.h"

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
    FString Endpoint = TEXT("tcp://127.0.0.1:5602");

    UPROPERTY(EditAnywhere, Category="ACESim")
    bool bAutoConnectOnBeginPlay = true;

    UPROPERTY(EditAnywhere, Category="ACESim")
    TArray<FName> RotorComponentNames;

private:
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
    TUniquePtr<class std::thread> ReceiveThread;
    FCriticalSection SampleMutex;
    FACESimLatestSample SharedSample;
    TAtomic<bool> bStopRequested = false;
    TAtomic<bool> bHasReceivedSample = false;
    TAtomic<int64> LastTimestampUs = 0;
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
    if (ReceiveThread.IsValid())
    {
        return;
    }

    bStopRequested = false;
    ReceiveThread = MakeUnique<std::thread>([this]() { ReceiverLoop(); });
}

void UACESimVehicleSyncComponent::StopReceiver()
{
    bStopRequested = true;
    if (ReceiveThread.IsValid())
    {
        ReceiveThread->join();
        ReceiveThread.Reset();
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

    while (!bStopRequested)
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
    if (!bHasReceivedSample)
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

    if (AACESimVehicleActor* VehicleActor = Cast<AACESimVehicleActor>(Owner))
    {
        VehicleActor->SetRotorCount(static_cast<int32>(LatestSample.RotorCount));
    }

    if (CachedRotorComponents.Num() == 0)
    {
        RefreshRotorComponents();
    }

    const int32 Count = FMath::Min<int32>(LatestSample.RotorCount, CachedRotorComponents.Num());
    for (int32 RotorIndex = 0; RotorIndex < Count; ++RotorIndex)
    {
        if (USceneComponent* RotorComponent = CachedRotorComponents[RotorIndex].Get())
        {
            const float RotorDegrees = FMath::RadiansToDegrees(
                static_cast<float>(LatestSample.RotorAngleRad[RotorIndex])
            );
            RotorComponent->SetRelativeRotation(FRotator(0.0f, RotorDegrees, 0.0f));
        }
    }
}
""",
    }


def write_templates(project_root: Path, overwrite: bool) -> None:
    for relative_path, content in _templates().items():
        target_path = project_root / relative_path
        target_path.parent.mkdir(parents=True, exist_ok=True)
        if target_path.exists() and not overwrite:
            raise FileExistsError(f"{target_path} already exists. Re-run with --overwrite to replace it.")
        target_path.write_text(content, encoding="utf-8")


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
    args = parser.parse_args()

    project_root = args.project_root.expanduser().resolve()
    project_root.mkdir(parents=True, exist_ok=True)
    write_templates(project_root, overwrite=args.overwrite)
    print(f"Generated UE5 project scaffold at {project_root}")


if __name__ == "__main__":
    main()
