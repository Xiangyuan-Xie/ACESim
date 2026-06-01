import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ACESIMUE_ROOT = ROOT / "acesim" / "third_party" / "unreal" / "ACESimUE"
REMOVED_PARENT_UE_TOOL_DIR = ROOT / "acesim" / "tools" / "ue5"
REQUIRED_UE_RUNTIME_ASSETS = [
    "Content/ACESim/x500_arm2x/visual_manifest.json",
    "Content/ACESim/x500_arm2x/base_link.uasset",
    "Content/ACESim/x500_arm2x/rotor_1.uasset",
    "Content/ACESim/Environment/Ground/Materials/M_ACESim_HelipadConcrete.uasset",
    "Content/ACESim/Environment/Ground/Textures/T_ACESim_HelipadTop_BaseColor.uasset",
    "Content/ACESim/Environment/Ground/SourceTextures/T_ACESim_polyhaven_noon_grass_hdri.hdr",
    "Content/ACESim/Environment/TestField/Meshes/SM_TestField_LandingPad.uasset",
    "Content/StarterContent/Materials/M_Basic_Wall.uasset",
    "Content/StarterContent/Shapes/Shape_Cube.uasset",
    "Content/StarterContent/Textures/T_Ground_Grass_D.uasset",
]
LFS_PATTERNS = [
    "*.uasset",
    "*.umap",
    "*.ubulk",
    "*.uexp",
    "*.tga",
    "*.hdr",
    "*.jpg",
    "*.png",
    "*.obj",
    "*.fbx",
]


def _bridge_source_root(project_root: Path) -> Path:
    return project_root / "Plugins" / "ACESimBridge" / "Source" / "ACESimBridge"


def test_unreal_project_is_managed_as_project_submodule_only() -> None:
    gitmodules = (ROOT / ".gitmodules").read_text(encoding="utf-8")

    assert '[submodule "acesim/third_party/unreal/ACESimUE"]' in gitmodules
    assert "path = acesim/third_party/unreal/ACESimUE" in gitmodules
    assert "url = git@github.com:Xiangyuan-Xie/ACESimUE.git" in gitmodules
    assert "branch = main" in gitmodules
    assert '[submodule "acesim/third_party/unreal/UnrealEngine"]' not in gitmodules
    assert "EpicGames/UnrealEngine" not in gitmodules


def test_acesimue_submodule_is_the_direct_working_project() -> None:
    project_root = ACESIMUE_ROOT

    field_cpp = (project_root / "Source" / "ACESimUE" / "ACESimOutdoorTestFieldActor.cpp").read_text(encoding="utf-8")
    assert "ResolveAcesimLightingPresetName" in field_cpp
    assert 'FParse::Value(FCommandLine::Get(), TEXT("ACESimLightingPreset="), PresetName)' in field_cpp
    assert 'const TCHAR* LightingPresetName = TEXT("golden_hour")' not in field_cpp
    assert 'const TCHAR* LightingPresetName = TEXT("cinematic_day")' not in field_cpp
    assert 'FCString::Strcmp(*LightingPresetName, TEXT("golden_hour"))' in field_cpp
    assert (project_root / ".gitignore").is_file()
    assert (project_root / "Tools" / "smoke_ue_bridge.sh").is_file()
    assert (project_root / "Tools" / "verify_ue_runtime_visual.py").is_file()
    assert (project_root / ".git").exists()


def test_acesimue_content_assets_are_migrated_into_submodule() -> None:
    missing_assets = [asset for asset in REQUIRED_UE_RUNTIME_ASSETS if not (ACESIMUE_ROOT / asset).is_file()]

    assert missing_assets == []


def test_acesimue_migrated_content_does_not_reference_retired_project_dir() -> None:
    retired_project_dir = "/home/xxy/ACESim-unreal" + "/projects/ACESimUE"

    for path in [
        ACESIMUE_ROOT / "Content" / "ACESim" / "x500_arm2x" / "import_acesim_assets.py",
        ACESIMUE_ROOT / "Content" / "ACESim" / "x500_arm2x" / "manifest.json",
        ACESIMUE_ROOT / "Content" / "ACESim" / "x500_arm2x" / "visual_manifest.json",
        ACESIMUE_ROOT / "Content" / "ACESim" / "Environment" / "Ground" / "import_acesim_ground_assets.py",
        ACESIMUE_ROOT / "Content" / "ACESim" / "Environment" / "Ground" / "ground_manifest.json",
        ACESIMUE_ROOT / "Content" / "ACESim" / "Environment" / "Ground" / "ground_pbr_manifest.json",
        ACESIMUE_ROOT / "Content" / "ACESim" / "Environment" / "TestField" / "import_acesim_testfield_assets.py",
    ]:
        assert retired_project_dir not in path.read_text(encoding="utf-8"), path


def test_acesimue_runtime_assets_are_lfs_tracked_and_not_gitignored() -> None:
    gitattributes = (ACESIMUE_ROOT / ".gitattributes").read_text(encoding="utf-8")

    for pattern in LFS_PATTERNS:
        assert f"{pattern} filter=lfs diff=lfs merge=lfs -text" in gitattributes

    for asset in [
        "Content/ACESim/x500_arm2x/base_link.uasset",
        "Content/ACESim/Environment/Ground/Materials/M_ACESim_HelipadConcrete.uasset",
        "Content/StarterContent/Materials/M_Basic_Wall.uasset",
    ]:
        result = subprocess.run(
            ["git", "-C", str(ACESIMUE_ROOT), "check-ignore", asset],
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode != 0, result.stdout


def test_parent_ue_tools_directory_is_removed() -> None:
    assert not REMOVED_PARENT_UE_TOOL_DIR.exists()


def test_acesimue_submodule_contains_bridge_plugin() -> None:
    project_root = ACESIMUE_ROOT

    plugin_root = project_root / "Plugins" / "ACESimBridge"
    source_root = _bridge_source_root(project_root)
    assert (project_root / "ACESimUE.uproject").is_file()
    assert (plugin_root / "ACESimBridge.uplugin").is_file()

    uproject = json.loads((project_root / "ACESimUE.uproject").read_text(encoding="utf-8"))
    uplugin = json.loads((plugin_root / "ACESimBridge.uplugin").read_text(encoding="utf-8"))
    assert uproject["Plugins"] == [{"Name": "ACESimBridge", "Enabled": True}]
    assert uplugin["Modules"] == [{"Name": "ACESimBridge", "Type": "Runtime", "LoadingPhase": "Default"}]

    assert (source_root / "Public" / "ACESimVehicleActor.h").is_file()
    assert (source_root / "Private" / "ACESimVehicleActor.cpp").is_file()
    assert (source_root / "Public" / "ACESimVehicleSyncComponent.h").is_file()
    assert (source_root / "Private" / "ACESimVehicleSyncComponent.cpp").is_file()
    assert (source_root / "Public" / "ACESimArmStateSyncComponent.h").is_file()
    assert (source_root / "Private" / "ACESimArmStateSyncComponent.cpp").is_file()
    assert (source_root / "Public" / "ACESimSensorFeedbackComponent.h").is_file()
    assert (source_root / "Private" / "ACESimSensorFeedbackComponent.cpp").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.h").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimPlayerController.h").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimPlayerController.cpp").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimGameViewportClient.h").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimGameViewportClient.cpp").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimOutdoorTestFieldActor.h").is_file()
    assert (project_root / "Source" / "ACESimUE" / "ACESimOutdoorTestFieldActor.cpp").is_file()

    target_cs = (project_root / "Source" / "ACESimUE.Target.cs").read_text(encoding="utf-8")
    editor_target_cs = (project_root / "Source" / "ACESimUEEditor.Target.cs").read_text(encoding="utf-8")
    assert "DefaultBuildSettings = BuildSettingsVersion.V6" in target_cs
    assert "DefaultBuildSettings = BuildSettingsVersion.V6" in editor_target_cs
    assert "EngineIncludeOrderVersion.Unreal5_7" in target_cs
    assert "EngineIncludeOrderVersion.Unreal5_7" in editor_target_cs


def test_ue5_scaffold_links_zmq_and_preserves_wire_size() -> None:
    project_root = ACESIMUE_ROOT

    source_root = _bridge_source_root(project_root)
    build_cs = (source_root / "ACESimBridge.Build.cs").read_text(encoding="utf-8")
    sync_cpp = (source_root / "Private" / "ACESimVehicleSyncComponent.cpp").read_text(encoding="utf-8")

    assert 'PublicSystemIncludePaths.Add("/usr/include");' in build_cs
    assert 'PublicSystemLibraryPaths.Add("/usr/lib/x86_64-linux-gnu");' in build_cs
    assert 'PublicSystemLibraries.Add("zmq");' in build_cs
    assert "static_assert(sizeof(FACESimVisualWireSample) == 196" in sync_cpp


def test_ue5_scaffold_reserves_disabled_sensor_feedback() -> None:
    project_root = ACESIMUE_ROOT

    source_root = _bridge_source_root(project_root)
    sensor_header = (source_root / "Public" / "ACESimSensorFeedbackComponent.h").read_text(encoding="utf-8")
    sensor_cpp = (source_root / "Private" / "ACESimSensorFeedbackComponent.cpp").read_text(encoding="utf-8")

    assert "UACESimSensorFeedbackComponent" in sensor_header
    assert "bEnableSensorFeedback = false" in sensor_header
    assert 'CameraRgbEndpoint = TEXT("tcp://127.0.0.1:5610")' in sensor_header
    assert 'DepthEndpoint = TEXT("tcp://127.0.0.1:5611")' in sensor_header
    assert 'SegmentationEndpoint = TEXT("tcp://127.0.0.1:5612")' in sensor_header
    assert 'EventEndpoint = TEXT("tcp://127.0.0.1:5613")' in sensor_header
    assert "FACESimSensorFrameHeader" in sensor_header
    assert "FACESimBridgeClock" in sensor_header
    assert "Phase 1 intentionally does not publish sensor payloads" in sensor_cpp


def test_ue5_vehicle_sync_uses_latest_sample_and_timeout() -> None:
    project_root = ACESIMUE_ROOT

    source_root = _bridge_source_root(project_root)
    sync_header = (source_root / "Public" / "ACESimVehicleSyncComponent.h").read_text(encoding="utf-8")
    sync_cpp = (source_root / "Private" / "ACESimVehicleSyncComponent.cpp").read_text(encoding="utf-8")

    assert "ZMQ_CONFLATE" in sync_cpp
    assert "ZMQ_RCVTIMEO" in sync_cpp
    assert "ReceivedBytes != sizeof(WireSample)" in sync_cpp
    assert "ConvertWorldVectorNwuToUe" in sync_cpp
    assert "ConvertAttitudeNwuFluToUe" in sync_cpp
    assert "class FACESimVehicleReceiverThread;" in sync_header
    assert "FACESimVehicleReceiverThread* ReceiveThread = nullptr" in sync_header
    assert "class FACESimVehicleReceiverThread" in sync_cpp
    assert "<thread>" not in sync_header
    assert "std::thread" not in sync_header
    assert "bLoggedFirstSample = false" in sync_header
    assert "bLoggedFirstApply = false" in sync_header
    assert "ACESim visual stream connected" in sync_cpp
    assert "ACESim visual state applied" in sync_cpp
    assert "VehicleActor->GetRotorComponentByIndex(RotorIndex)" in sync_cpp
    assert "SetWorldOriginOffsetCm" in sync_header
    assert "uint64 LastAppliedTimestampUs = 0" in sync_header
    assert "if (LatestSample.TimestampUs == LastAppliedTimestampUs)" in sync_cpp
    assert "LastAppliedTimestampUs = LatestSample.TimestampUs" in sync_cpp
    assert "WorldOriginOffsetCm + LatestSample.PositionCm" in sync_cpp
    assert "Owner->SetActorLocationAndRotation(\n        LatestSample.PositionCm" not in sync_cpp


def test_ue5_vehicle_actor_is_visible_and_self_syncing() -> None:
    project_root = ACESIMUE_ROOT

    source_root = _bridge_source_root(project_root)
    actor_header = (source_root / "Public" / "ACESimVehicleActor.h").read_text(encoding="utf-8")
    actor_cpp = (source_root / "Private" / "ACESimVehicleActor.cpp").read_text(encoding="utf-8")

    assert "UACESimVehicleSyncComponent" in actor_header
    assert "UACESimArmStateSyncComponent" in actor_header
    assert "UStaticMeshComponent" in actor_header
    assert "TObjectPtr<UACESimVehicleSyncComponent> SyncComponent" in actor_header
    assert "TObjectPtr<UACESimArmStateSyncComponent> ArmStateSyncComponent" in actor_header
    assert "TMap<FName, TObjectPtr<USceneComponent>> BodyComponents" in actor_header
    assert "TMap<FName, TObjectPtr<UStaticMeshComponent>> MeshComponents" in actor_header
    assert "TMap<FName, FACESimJointRuntime> JointRuntimes" in actor_header
    assert "SetWorldOriginOffsetCm" in actor_header
    assert 'CreateDefaultSubobject<UACESimVehicleSyncComponent>(TEXT("ACESimVehicleSync"))' in actor_cpp
    assert 'CreateDefaultSubobject<UACESimArmStateSyncComponent>(TEXT("ACESimArmStateSync"))' in actor_cpp
    assert "SyncComponent->SetWorldOriginOffsetCm" in actor_cpp
    assert "LoadAcesimVisualManifest" in actor_cpp
    assert "BuildVisualTreeFromManifest" in actor_cpp
    assert "/Game/ACESim/x500_arm2x/visual_manifest" in actor_cpp
    assert "RotorCircleRadiusCm" not in actor_header
    assert "JointLocations" not in actor_cpp
    assert "FMath::Cos(Angle)" not in actor_cpp
    assert "ConstructorHelpers::FObjectFinder<UStaticMesh> RotorMesh" not in actor_cpp
    assert "SetCollisionEnabled(ECollisionEnabled::NoCollision)" in actor_cpp
    assert "ACESim real vehicle mesh loaded" in actor_cpp
    assert "ACESim visual manifest loaded" in actor_cpp
    assert "ACESim vehicle mesh missing; using fallback proxy" in actor_cpp
    assert "SetRelativeLocation" in actor_cpp


def test_ue5_scaffold_generates_arm_state_sync_component() -> None:
    project_root = ACESIMUE_ROOT

    source_root = _bridge_source_root(project_root)
    arm_header = (source_root / "Public" / "ACESimArmStateSyncComponent.h").read_text(encoding="utf-8")
    arm_cpp = (source_root / "Private" / "ACESimArmStateSyncComponent.cpp").read_text(encoding="utf-8")

    assert "class USceneComponent;" in arm_header
    assert 'Endpoint = TEXT("tcp://127.0.0.1:5603")' in arm_header
    assert "static_assert(sizeof(FACESimLegacyArmStateWireSample) == 128" in arm_cpp
    assert "static_assert(sizeof(FACESimArmStateWireSample) == 176" in arm_cpp
    assert 'JointComponentNames.Add(TEXT("joint_gripper_left"))' in arm_cpp
    assert 'JointComponentNames.Add(TEXT("joint_gripper_right"))' in arm_cpp
    assert "PositionRad[JointIndex]" in arm_cpp
    assert "ApplyArmJointState" in arm_cpp
    assert "uint64 LastAppliedTimestampUs = 0" in arm_header
    assert "if (LatestSample.TimestampUs == LastAppliedTimestampUs)" in arm_cpp
    assert "LastAppliedTimestampUs = LatestSample.TimestampUs" in arm_cpp
    assert "ACESim arm state stream connected" in arm_cpp
    assert "7-joint" in arm_cpp
    assert "5-joint" in arm_cpp


def test_ue5_project_uses_free_camera_without_default_pawn_sphere() -> None:
    project_root = ACESIMUE_ROOT

    game_mode_header = (project_root / "Source" / "ACESimUE" / "ACESimUEGameMode.h").read_text(encoding="utf-8")
    game_mode_cpp = (project_root / "Source" / "ACESimUE" / "ACESimUEGameMode.cpp").read_text(encoding="utf-8")
    free_camera_header = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.h").read_text(encoding="utf-8")
    free_camera_cpp = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").read_text(encoding="utf-8")
    player_controller_header = (project_root / "Source" / "ACESimUE" / "ACESimPlayerController.h").read_text(
        encoding="utf-8"
    )
    player_controller_cpp = (project_root / "Source" / "ACESimUE" / "ACESimPlayerController.cpp").read_text(
        encoding="utf-8"
    )
    viewport_cpp = (project_root / "Source" / "ACESimUE" / "ACESimGameViewportClient.cpp").read_text(encoding="utf-8")
    default_engine = (project_root / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")

    assert "AACESimUEGameMode" in game_mode_header
    assert "AACESimFreeCameraPawn" in game_mode_cpp
    assert "AACESimPlayerController" in game_mode_cpp
    assert "DefaultPawnClass = AACESimFreeCameraPawn::StaticClass()" in game_mode_cpp
    assert "PlayerControllerClass = AACESimPlayerController::StaticClass()" in game_mode_cpp
    assert "SetTimerForNextTick" in game_mode_cpp
    assert "ASpectatorPawn" not in game_mode_cpp
    assert "bStartPlayersAsSpectators = false" in game_mode_cpp
    assert "ACESim free camera enabled" in game_mode_cpp
    assert "ACESim UE scene ready for visual stream" in game_mode_cpp
    assert "SetViewTarget(VehicleActor)" not in game_mode_cpp
    assert "SetViewTarget(*It)" not in game_mode_cpp
    assert "UFloatingPawnMovement" in free_camera_header
    assert "UCameraComponent" in free_camera_header
    assert "FrameVehicle(AActor* VehicleActor)" in free_camera_header
    assert "FrameVehicleShadowCheck(AActor* VehicleActor, const FVector& EnvironmentAnchor)" in free_camera_header
    assert "SetOrbitTargetFromActor(AActor* VehicleActor)" in free_camera_header
    assert "ApplyOrbitMouseDelta(const FVector2D& Delta)" in free_camera_header
    assert "PanOrbitTarget(const FVector2D& Delta)" in free_camera_header
    assert "ZoomOrbit(float WheelDelta)" in free_camera_header
    assert "virtual void Tick(float DeltaSeconds) override" not in free_camera_header
    assert "CaptureLook" not in free_camera_cpp
    assert "ReleaseLook" in free_camera_cpp
    assert "FInputModeGameOnly" not in free_camera_cpp
    assert "GetMousePosition" not in free_camera_cpp
    assert "IsInputKeyDown(EKeys::RightMouseButton)" not in free_camera_cpp
    assert "IsInputKeyDown(EKeys::LeftMouseButton)" not in free_camera_cpp
    assert "ACESim camera drag enabled" not in free_camera_cpp
    assert "UpdateOrbitCamera()" in free_camera_cpp
    assert "OrbitTarget" in free_camera_cpp
    assert "OrbitDistanceCm" in free_camera_cpp
    assert "FRotationMatrix::MakeFromX(OrbitTarget - CameraLocation)" in free_camera_cpp
    assert "SetActorRotation(FRotator(PitchDegrees, YawDegrees, 0.0f))" not in free_camera_cpp
    assert "MoveUp" in free_camera_cpp
    assert "YawOnlyFrame(FRotator(0.0f, GetActorRotation().Yaw, 0.0f))" in free_camera_cpp
    assert "EKeys::Escape" in free_camera_cpp
    assert "FrameVehicle(VehicleActor)" in game_mode_cpp
    assert "GetComponentsBoundingBox(true)" in free_camera_cpp
    assert "FRotationMatrix::MakeFromX" in free_camera_cpp
    assert "virtual void PlayerTick(float DeltaTime) override" in player_controller_header
    assert "virtual void OnPossess(APawn* InPawn) override" in player_controller_header
    assert "FSlateApplication::Get()" not in player_controller_cpp
    assert "GetPressedMouseButtons()" not in player_controller_cpp
    assert "GetCursorPos()" not in player_controller_cpp
    assert "GetMousePosition" not in player_controller_cpp
    assert "EKeys::RightMouseButton" not in player_controller_cpp
    assert "EKeys::LeftMouseButton" not in player_controller_cpp
    assert "bShowMouseCursor = true" in player_controller_cpp
    assert "FInputModeGameAndUI" in player_controller_cpp
    assert "EMouseLockMode::DoNotLock" in player_controller_cpp
    assert "SetHideCursorDuringCapture(false)" in player_controller_cpp
    assert "FInputModeGameOnly" not in player_controller_cpp
    assert "FInputModeGameOnly" not in viewport_cpp
    assert "ACESim player controller ready" in player_controller_cpp
    assert "ACESim camera pawn possessed" in player_controller_cpp
    assert "ACESim camera drag started: button=%s" not in player_controller_cpp
    assert "ACESim camera drag delta applied" not in player_controller_cpp
    assert "ACESim camera drag ended" not in player_controller_cpp
    assert "void UACESimGameViewportClient::Tick(float DeltaTime)" in viewport_cpp
    assert "FSlateApplication::Get().GetCursorPos()" in viewport_cpp
    assert "GetPressedMouseButtons()" in viewport_cpp
    assert "ACESim camera tick drag started" in viewport_cpp
    assert "ACESim camera tick delta applied" in viewport_cpp
    assert "EMouseCaptureMode::CaptureDuringMouseDown" in viewport_cpp
    assert "Viewport->CaptureMouse(true)" in viewport_cpp
    assert "Viewport->CaptureMouse(false)" in viewport_cpp
    assert "/Engine/Maps/Templates/Template_Default" not in default_engine
    assert "GameDefaultMap=/Engine/Maps/Entry" in default_engine
    assert "Grasslands" not in default_engine


def test_ue5_free_camera_has_reliable_right_mouse_capture_and_keyboard_look_fallback() -> None:
    project_root = ACESIMUE_ROOT

    free_camera_header = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.h").read_text(encoding="utf-8")
    free_camera_cpp = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").read_text(encoding="utf-8")
    player_controller_header = (project_root / "Source" / "ACESimUE" / "ACESimPlayerController.h").read_text(
        encoding="utf-8"
    )
    player_controller_cpp = (project_root / "Source" / "ACESimUE" / "ACESimPlayerController.cpp").read_text(
        encoding="utf-8"
    )
    viewport_header = (project_root / "Source" / "ACESimUE" / "ACESimGameViewportClient.h").read_text(encoding="utf-8")
    viewport_cpp = (project_root / "Source" / "ACESimUE" / "ACESimGameViewportClient.cpp").read_text(encoding="utf-8")
    default_input = (project_root / "Config" / "DefaultInput.ini").read_text(encoding="utf-8")
    default_engine = (project_root / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")

    assert "ApplyKeyboardLookDelta" in free_camera_header
    assert "SetOrbitTargetFromActor(AActor* VehicleActor)" in free_camera_header
    assert "ApplyOrbitMouseDelta(const FVector2D& Delta)" in free_camera_header
    assert "PanOrbitTarget(const FVector2D& Delta)" in free_camera_header
    assert "ZoomOrbit(float WheelDelta)" in free_camera_header
    assert "LookYaw" in free_camera_cpp
    assert "LookPitch" in free_camera_cpp
    assert '+AxisMappings=(AxisName="LookYaw",Key=Right,Scale=1.000000)' in default_input
    assert '+AxisMappings=(AxisName="LookYaw",Key=Left,Scale=-1.000000)' in default_input
    assert '+AxisMappings=(AxisName="LookPitch",Key=Up,Scale=1.000000)' in default_input
    assert '+AxisMappings=(AxisName="LookPitch",Key=Down,Scale=-1.000000)' in default_input
    assert "virtual bool InputKey(const FInputKeyEventArgs& Params) override" in player_controller_header
    assert "StartRightMouseCapture()" not in player_controller_cpp
    assert "FinishRightMouseCapture()" not in player_controller_cpp
    assert "InputKey(const FInputKeyEventArgs& EventArgs)" in viewport_header
    assert "virtual bool InputAxis(const FInputKeyEventArgs& EventArgs) override" in viewport_header
    assert "virtual void Tick(float DeltaTime) override" in viewport_header
    assert "EventArgs.Key == EKeys::LeftMouseButton" in viewport_cpp
    assert "EventArgs.Key == EKeys::RightMouseButton" in viewport_cpp
    assert "GetPressedMouseButtons()" in viewport_cpp
    assert "PressedButtons.Contains(EKeys::LeftMouseButton)" in viewport_cpp
    assert "PressedButtons.Contains(EKeys::RightMouseButton)" in viewport_cpp
    assert "FSlateApplication::Get().GetCursorPos()" in viewport_cpp
    assert "EventArgs.Key == EKeys::MouseX" not in viewport_cpp
    assert "EventArgs.Key == EKeys::MouseY" not in viewport_cpp
    assert "EventArgs.Key == EKeys::MouseWheelAxis" in viewport_cpp
    assert "bool UACESimGameViewportClient::InputAxis(const FInputKeyEventArgs& EventArgs)" in viewport_cpp
    assert "HandleMouseWheel(EventArgs.AmountDepressed)" in viewport_cpp
    assert "bool UACESimGameViewportClient::HandleMouseWheel(float WheelDelta)" in viewport_cpp
    assert "Delta.X = EventArgs.AmountDepressed" not in viewport_cpp
    assert "Delta.Y = EventArgs.AmountDepressed" not in viewport_cpp
    assert "EMouseCaptureMode::CaptureDuringMouseDown" in viewport_cpp
    assert "SetMouseLockMode(EMouseLockMode::DoNotLock)" in viewport_cpp
    assert "Viewport->CaptureMouse(true)" in viewport_cpp
    assert "Viewport->LockMouseToViewport(true)" not in viewport_cpp
    assert "Viewport->CaptureMouse(false)" in viewport_cpp
    assert "Viewport->LockMouseToViewport(false)" in viewport_cpp
    assert "ApplyOrbitMouseDelta(CursorDelta)" in viewport_cpp
    assert "PanOrbitTarget(CursorDelta)" in viewport_cpp
    assert "ZoomOrbit(WheelDelta)" in viewport_cpp
    assert "ACESim camera wheel zoom applied" in viewport_cpp
    assert "ACESim camera tick drag started" in viewport_cpp
    assert "ACESim camera tick drag ended" in viewport_cpp
    assert "ACESim camera tick delta applied" in viewport_cpp
    input_key_block = viewport_cpp.split("void UACESimGameViewportClient::Tick(float DeltaTime)", 1)[0]
    assert "StartCameraCapture" not in input_key_block
    assert "ACESim viewport input diagnostic" in viewport_cpp
    assert "FInputModeGameOnly" not in player_controller_cpp
    assert "FInputModeGameOnly" not in viewport_cpp
    assert "GameViewportClientClassName=/Script/ACESimUE.ACESimGameViewportClient" in default_engine
    forbidden_drag_log = (
        "ApplyReleasedInputMode();\n"
        "        UE_LOG(\n"
        "            LogTemp,\n"
        "            Display,\n"
        '            TEXT("ACESim camera drag started'
    )
    assert forbidden_drag_log not in player_controller_cpp


def test_ue5_vehicle_actor_does_not_double_apply_manifest_root_body_transform() -> None:
    project_root = ACESIMUE_ROOT

    actor_cpp = (_bridge_source_root(project_root) / "Private" / "ACESimVehicleActor.cpp").read_text(encoding="utf-8")

    assert 'GetStringField(*ManifestRootObject, TEXT("name"))' in actor_cpp
    assert "const bool bIsManifestRootBody = BodyName == ManifestRootBodyName" in actor_cpp
    assert "BodyComponent->SetRelativeLocation(FVector::ZeroVector)" in actor_cpp
    assert "BodyComponent->SetRelativeRotation(FRotator::ZeroRotator)" in actor_cpp


def test_ue5_vehicle_actor_uses_mjcf_home_pose_and_axis_handedness() -> None:
    project_root = ACESIMUE_ROOT

    actor_cpp = (_bridge_source_root(project_root) / "Private" / "ACESimVehicleActor.cpp").read_text(encoding="utf-8")

    assert "ApplyArmMountPitchCorrection" not in actor_cpp
    assert "PitchCorrection" not in actor_cpp
    assert "ACESim arm mount visual pitch correction applied" not in actor_cpp
    assert "JsonHingeAxisField" in actor_cpp
    assert "return -JsonVectorField" in actor_cpp
    assert 'JsonDoubleField(*JointObject, TEXT("home_position"), 0.0)' in actor_cpp
    assert "ApplyManifestHomeJointStates()" in actor_cpp
    assert "void AACESimVehicleActor::ApplyManifestHomeJointStates()" in actor_cpp
    assert "ApplyArmJointState(Pair.Key, Pair.Value.HomePositionRad)" in actor_cpp
    assert "const FVector SlideAxis = HomeQuat.RotateVector(Axis)" in actor_cpp
    assert "HomeLocation + SlideAxis * static_cast<float>(Position * MToCm)" in actor_cpp
    assert "ACESim MJCF home arm pose applied" in actor_cpp
    assert "ACESim arm debug axes enabled" in actor_cpp


def test_ue5_project_disables_mouse_capture_on_launch() -> None:
    project_root = ACESIMUE_ROOT

    default_input = (project_root / "Config" / "DefaultInput.ini").read_text(encoding="utf-8")

    assert "bCaptureMouseOnLaunch=False" in default_input
    assert "DefaultViewportMouseCaptureMode=CaptureDuringMouseDown" in default_input
    assert "DefaultViewportMouseLockMode=DoNotLock" in default_input
    assert '+ActionMappings=(ActionName="ReleaseLook",Key=Escape)' in default_input
    assert '+AxisMappings=(AxisName="MoveForward",Key=W,Scale=1.000000)' in default_input
    assert '+AxisMappings=(AxisName="MoveRight",Key=D,Scale=1.000000)' in default_input
    assert '+AxisMappings=(AxisName="MoveUp",Key=SpaceBar,Scale=1.000000)' in default_input
    assert '+AxisMappings=(AxisName="MoveUp",Key=LeftControl,Scale=-1.000000)' in default_input
    assert 'AxisName="Turn",Key=MouseX' not in default_input
    assert 'AxisName="LookUp",Key=MouseY' not in default_input


def test_ue5_project_auto_spawns_vehicle_actor_on_play() -> None:
    project_root = ACESIMUE_ROOT

    game_mode_header = (project_root / "Source" / "ACESimUE" / "ACESimUEGameMode.h").read_text(encoding="utf-8")
    game_mode_cpp = (project_root / "Source" / "ACESimUE" / "ACESimUEGameMode.cpp").read_text(encoding="utf-8")
    build_cs = (project_root / "Source" / "ACESimUE" / "ACESimUE.Build.cs").read_text(encoding="utf-8")
    default_engine = (project_root / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")
    default_game = (project_root / "Config" / "DefaultGame.ini").read_text(encoding="utf-8")

    assert "AACESimUEGameMode" in game_mode_header
    assert '#include "ACESimVehicleActor.h"' in game_mode_cpp
    assert "TActorIterator<AACESimVehicleActor>" in game_mode_cpp
    assert "SpawnActor<AACESimVehicleActor>" in game_mode_cpp
    assert "SetViewTarget(VehicleActor)" not in game_mode_cpp
    assert '"ACESimBridge"' in build_cs
    assert "EditorStartupMap=/Engine/Maps/Entry" in default_engine
    assert "GameDefaultMap=/Engine/Maps/Entry" in default_engine
    assert "/Engine/Maps/Templates/Template_Default" not in default_engine
    assert "Grasslands" not in default_engine
    assert "OpenWorld" not in default_engine
    assert "GlobalDefaultGameMode=/Script/ACESimUE.ACESimUEGameMode" in default_engine
    assert '+DirectoriesToAlwaysCook=(Path="/Game/ACESim/x500_arm2x")' in default_game
    assert "Grasslands" not in default_game


def test_ue5_project_generates_lightweight_outdoor_test_field() -> None:
    project_root = ACESIMUE_ROOT

    game_mode_cpp = (project_root / "Source" / "ACESimUE" / "ACESimUEGameMode.cpp").read_text(encoding="utf-8")
    field_header = (project_root / "Source" / "ACESimUE" / "ACESimOutdoorTestFieldActor.h").read_text(encoding="utf-8")
    field_cpp = (project_root / "Source" / "ACESimUE" / "ACESimOutdoorTestFieldActor.cpp").read_text(encoding="utf-8")
    build_cs = (project_root / "Source" / "ACESimUE" / "ACESimUE.Build.cs").read_text(encoding="utf-8")
    default_game = (project_root / "Config" / "DefaultGame.ini").read_text(encoding="utf-8")

    assert '#include "ACESimOutdoorTestFieldActor.h"' in game_mode_cpp
    assert "TActorIterator<AACESimOutdoorTestFieldActor>" in game_mode_cpp
    assert "SpawnActor<AACESimOutdoorTestFieldActor>" in game_mode_cpp
    assert "ACESim outdoor test field spawned" in game_mode_cpp
    assert "AACESimOutdoorTestFieldActor" in field_header
    assert "GroundPlaneComponent" in field_header
    assert "SkyDomeComponent" in field_header
    assert "SkyDomeMaterialInstance" in field_header
    assert "CenterMarkerComponent" not in field_header
    assert "LandingPadTopComponent" in field_header
    assert "LandingPadSideComponent" in field_header
    assert "LandingPadWhiteMarkingComponent" not in field_header
    assert "LandingPadYellowMarkingComponent" not in field_header
    assert "LandingPadWearComponent" not in field_header
    assert "GridLineComponents" in field_header
    assert "HelipadAnchorComponent" in field_header
    assert "PrimaryActorTick.bCanEverTick = false" in field_cpp
    assert "UInstancedStaticMeshComponent" not in field_header
    assert "UInstancedStaticMeshComponent" not in field_cpp
    assert "UDirectionalLightComponent" in field_header
    assert "USkyLightComponent" in field_header
    assert "USkyAtmosphereComponent" in field_header
    assert "UExponentialHeightFogComponent" in field_header
    assert "UVolumetricCloudComponent" in field_header
    assert "CreateDefaultSubobject<UDirectionalLightComponent>" in field_cpp
    assert "CreateDefaultSubobject<USkyLightComponent>" in field_cpp
    assert "CreateDefaultSubobject<USkyAtmosphereComponent>" in field_cpp
    assert '#include "Components/SkyAtmosphereComponent.h"' in field_cpp
    assert "CreateDefaultSubobject<UExponentialHeightFogComponent>" in field_cpp
    assert "CreateDefaultSubobject<UVolumetricCloudComponent>" in field_cpp
    assert "m_SimpleVolumetricCloud_Inst" in field_cpp
    assert "SunLight->SetRelativeRotation(FRotator(58.0f, -38.0f, 0.0f))" in field_cpp
    assert "SunLight->SetIntensity(85000.0f)" not in field_cpp
    assert "SunLight->SetIntensity(12.0f)" not in field_cpp
    assert "SunLight->SetIntensity(11.0f)" in field_cpp
    assert "SunLight->SetAtmosphereSunLight(true)" in field_cpp
    assert "SunLight->SetAtmosphereSunLightIndex(0)" in field_cpp
    assert "SunLight->SetLightSourceAngle(0.5357f)" in field_cpp
    assert "SunLight->SetLightColor(FLinearColor(1.0f, 0.96f, 0.88f), true)" not in field_cpp
    assert "SunLight->SetLightColor(FLinearColor(1.0f, 0.98f, 0.94f), true)" in field_cpp
    assert "SunLight->ContactShadowLength = 18.0f" in field_cpp
    assert "SunLight->SetVolumetricScatteringIntensity(0.65f)" in field_cpp
    assert "SkyLight->SetIntensity(1.15f)" in field_cpp
    assert "SkyLight->SetRealTimeCapture(false)" in field_cpp
    assert "UPostProcessComponent" in field_header
    assert "CreateDefaultSubobject<UPostProcessComponent>" in field_cpp
    assert "PostProcess->Settings.bOverride_AutoExposureMinBrightness = true" in field_cpp
    assert "PostProcess->Settings.AutoExposureMethod = AEM_Manual" not in field_cpp
    assert "PostProcess->Settings.AutoExposureMethod = AEM_Basic" in field_cpp
    assert "PostProcess->Settings.AutoExposureMinBrightness = 11.2f" not in field_cpp
    assert "PostProcess->Settings.AutoExposureMaxBrightness = 11.2f" not in field_cpp
    assert "PostProcess->Settings.AutoExposureMinBrightness = 1.0f" in field_cpp
    assert "PostProcess->Settings.AutoExposureMaxBrightness = 1.0f" in field_cpp
    assert "PostProcess->Settings.AutoExposureBias = 0.18f" in field_cpp
    assert "PostProcess->Settings.bOverride_WhiteTemp = true" in field_cpp
    assert "PostProcess->Settings.WhiteTemp = 6500.0f" in field_cpp
    assert "PostProcess->Settings.BloomIntensity = 0.08f" in field_cpp
    assert "PostProcess->Settings.LocalExposureShadowContrastScale = 0.54f" in field_cpp
    assert "PostProcess->Settings.FilmSlope = 1.02f" in field_cpp
    assert "PostProcess->Settings.FilmToe = 0.40f" in field_cpp
    assert "PostProcess->Settings.FilmShoulder = 0.24f" in field_cpp
    assert "PostProcess->Settings.LocalExposureHighlightContrastScale = 0.50f" in field_cpp
    assert "PostProcess->Settings.LocalExposureDetailStrength = 0.24f" in field_cpp
    assert "PostProcess->Settings.ColorSaturation = FVector4(1.03f, 1.04f, 1.02f, 1.0f)" in field_cpp
    assert "PostProcess->Settings.ColorContrast = FVector4(1.05f, 1.05f, 1.04f, 1.0f)" in field_cpp
    assert "M_ACESim_SkyHDRI" not in field_cpp
    assert "LoadSkyHdriMaterial" not in field_cpp
    assert "SkyDomeComponent = CreateDefaultSubobject<UStaticMeshComponent>" in field_cpp
    assert "/Engine/EngineSky/SM_SkySphere.SM_SkySphere" in field_cpp
    assert "/Engine/EngineSky/M_Sky_Panning_Clouds2.M_Sky_Panning_Clouds2" in field_cpp
    assert 'SetVectorParameterValue(TEXT("Zenith Color"), FLinearColor(0.62f, 0.86f, 1.55f, 1.0f))' in field_cpp
    assert 'SetVectorParameterValue(TEXT("Horizon color"), FLinearColor(1.10f, 1.22f, 1.52f, 1.0f))' in field_cpp
    assert 'SetVectorParameterValue(TEXT("Overall Color"), FLinearColor(1.25f, 1.32f, 1.48f, 1.0f))' in field_cpp
    assert 'SetScalarParameterValue(TEXT("Cloud opacity"), 0.04f)' in field_cpp
    assert 'SetScalarParameterValue(TEXT("Sun brightness"), 0.55f)' in field_cpp
    assert 'SetScalarParameterValue(TEXT("Sun height"), 0.62f)' in field_cpp
    assert "ACESim clear sky dome loaded" in field_cpp
    assert "LoadSkyHdriCubemap" in field_cpp
    assert "T_ACESim_polyhaven_noon_grass_hdri" in field_cpp
    assert "SkyLight->SourceType = SLS_SpecifiedCubemap" in field_cpp
    assert "SkyLight->SetCubemap(SkyHdriCubemap)" in field_cpp
    assert "SkyLight->RecaptureSky()" in field_cpp
    assert "SkyAtmosphere->SetMieScatteringScale(0.08f)" in field_cpp
    assert "SkyAtmosphere->SetMieAnisotropy(0.72f)" in field_cpp
    assert "SkyAtmosphere->SetSkyLuminanceFactor(FLinearColor(1.45f, 1.55f, 1.80f, 1.0f))" in field_cpp
    assert "SkyAtmosphere->SetHeightFogContribution(0.10f)" in field_cpp
    assert "HeightFog->SetFogDensity(0.0011f)" not in field_cpp
    assert "HeightFog->SetFogDensity(0.00009f)" in field_cpp
    assert "HeightFog->SetFogHeightFalloff(0.020f)" in field_cpp
    assert "HeightFog->SetFogInscatteringColor(FLinearColor(0.72f, 0.78f, 0.84f, 1.0f))" in field_cpp
    assert "HeightFog->SetVolumetricFog(false)" in field_cpp
    assert "VolumetricCloud->SetMaterial(CloudMaterial)" in field_cpp
    assert "VolumetricCloud->SetVisibility(false)" in field_cpp
    assert "VolumetricCloud->SetSkyLightCloudBottomOcclusion(0.02f)" in field_cpp
    assert 'FCString::Strcmp(*LightingPresetName, TEXT("golden_hour"))' in field_cpp
    assert "SunLight->SetRelativeRotation(FRotator(18.0f, -36.0f, 0.0f))" in field_cpp
    assert "SunLight->SetLightColor(FLinearColor(1.0f, 0.72f, 0.42f), true)" in field_cpp
    assert "VolumetricCloud->SetVisibility(true)" in field_cpp
    assert "ACESim cinematic outdoor lighting ready" in field_cpp
    assert "ACESim volumetric cloud material loaded" in field_cpp
    assert "ACESim realistic sky and sunlight ready" not in field_cpp
    assert "SunLight->SetMobility(EComponentMobility::Movable)" in field_cpp
    assert "SunLight->ContactShadowLengthInWS = true" in field_cpp
    assert "ACESim dynamic aircraft shadows ready" in field_cpp
    assert "PlaneExtentCm = 10000.0f" in field_cpp
    assert "GroundPlaneComponent->SetWorldScale3D(FVector(200.0f, 200.0f, 1.0f))" in field_cpp
    assert "LoadShortGrassGroundMaterial" in field_header
    assert "LoadShortGrassGroundMaterial" in field_cpp
    assert "LoadHelipadTopMaterial" in field_header
    assert "LoadHelipadTopMaterial" in field_cpp
    assert "LoadHelipadConcreteSideMaterial" in field_header
    assert "LoadHelipadConcreteSideMaterial" in field_cpp
    assert "LoadHelipadPaintWhiteMaterial" not in field_header
    assert "LoadHelipadPaintYellowMaterial" not in field_header
    assert "LoadHelipadWearMaterial" not in field_header
    assert "/Game/ACESim/Environment/Ground/Materials/M_ACESim_ShortGrassGround.M_ACESim_ShortGrassGround" in field_cpp
    assert "/Game/ACESim/Environment/Ground/Materials/M_ACESim_HelipadTop.M_ACESim_HelipadTop" in field_cpp
    assert (
        "/Game/ACESim/Environment/Ground/Materials/M_ACESim_HelipadConcreteSide.M_ACESim_HelipadConcreteSide"
        in field_cpp
    )
    assert "M_ACESim_HelipadPaintWhite" not in field_cpp
    assert "M_ACESim_HelipadPaintYellow" not in field_cpp
    assert "M_ACESim_HelipadWearDark" not in field_cpp
    assert "ACESim short grass ground material loaded" in field_cpp
    assert "ACESim helipad top material loaded" in field_cpp
    assert "ACESim Isaac-style ground plane ready" in field_cpp
    assert "ACESim civil concrete helipad visual ready" in field_cpp
    assert "LandingPadRadiusCm = 350.0f" in field_cpp
    assert "LandingPadHeightCm = 4.0f" in field_cpp
    assert "LandingPadTopMeshPath" in field_cpp
    assert "LandingPadSideMeshPath" in field_cpp
    assert "HelipadWhiteMarkingsMeshPath" not in field_cpp
    assert "HelipadYellowMarkingsMeshPath" not in field_cpp
    assert "HelipadWearMeshPath" not in field_cpp
    assert "SM_TestField_LandingPad.SM_TestField_LandingPad" in field_cpp
    assert "SM_TestField_LandingPadSide.SM_TestField_LandingPadSide" in field_cpp
    assert "SM_TestField_HelipadMarkingsWhite" not in field_cpp
    assert "SM_TestField_HelipadMarkingsYellow" not in field_cpp
    assert "SM_TestField_HelipadWear" not in field_cpp
    assert "/Game/ACESim/Environment/TestField/Meshes/SM_TestField_LandingPad" in field_cpp
    assert "/Game/ACESim/Environment/TestField/Model/SM_TestField_LandingPad" not in field_cpp
    assert "LandingPadTopComponent->SetRelativeLocation(FVector(0.0f, 0.0f, LandingPadHeightCm))" in field_cpp
    assert "LandingPadHeightCm + 0.15f" not in field_cpp
    assert "LandingPadHeightCm + 0.20f" not in field_cpp
    assert "LandingPadHeightCm + 0.25f" not in field_cpp
    assert "ACESim landing pad visual aligned with MuJoCo: top_z_cm=4.0" in field_cpp
    assert "BuildReferenceGrid" in field_cpp
    assert "CreateGridLineComponent" in field_cpp
    assert "Grasslands" not in field_cpp
    assert "heliport_manifest.json" not in field_cpp
    assert "airport_manifest.json" not in field_cpp
    assert "FAssetData" not in field_cpp
    assert "GetAssetsByPath" not in field_cpp
    assert "HelipadAnchorComponent->SetRelativeLocation(FVector::ZeroVector)" in field_cpp
    assert "GetHelipadAnchorLocation" in field_cpp
    assert "ACESim-controlled helipad anchor" in field_cpp
    assert "GroundPlaneComponent->SetCastShadow(false)" in field_cpp
    assert "GroundPlaneComponent->SetCollisionEnabled(ECollisionEnabled::QueryOnly)" in field_cpp
    assert "/Game/ACESim/Environment/Shapes/Shape_Plane.Shape_Plane" not in field_cpp
    assert "/Game/ACESim/Environment/Shapes/Shape_Cube.Shape_Cube" not in field_cpp
    assert "/Game/ACESim/Environment/Shapes/Shape_Cylinder.Shape_Cylinder" not in field_cpp
    assert "/Game/ACESim/Environment/Props/SM_Rock.SM_Rock" not in field_cpp
    assert "/Game/ACESim/Environment/Props/SM_Bush.SM_Bush" not in field_cpp
    assert "/Game/ACESim/Environment/Materials/M_Ground_Grass.M_Ground_Grass" not in field_cpp
    assert "/Game/ACESim/Environment/Materials/M_Ground_Gravel.M_Ground_Gravel" not in field_cpp
    assert "/Game/ACESim/Environment/Materials/M_Concrete_Poured.M_Concrete_Poured" not in field_cpp
    assert "/Game/ACESim/Environment/Materials/M_Concrete_Grime.M_Concrete_Grime" not in field_cpp
    assert "/Game/ACESim/Environment/Materials/M_Concrete_Panels.M_Concrete_Panels" not in field_cpp
    assert "/Game/ACESim/Environment/Materials/M_Metal_Rust.M_Metal_Rust" not in field_cpp
    assert "/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial" not in field_cpp
    assert "/Engine/EngineMaterials/WorldGridMaterial.WorldGridMaterial" not in field_cpp
    assert "/Engine/OpenWorldTemplate/LandscapeMaterial/MI_ProcGrid.MI_ProcGrid" not in field_cpp
    assert "ACESim outdoor field materials loaded" not in field_cpp
    assert "ACESim offline test field meshes loaded" not in field_cpp
    assert "ACESim outdoor field asset missing" not in field_cpp
    assert "AllAssetsLoaded" not in field_cpp
    assert "UE_LOG(LogTemp, Error" not in field_cpp
    assert "SpawnActor<AActor>" not in field_cpp
    assert '"Slate"' in build_cs
    assert '"AssetRegistry"' not in build_cs
    assert '+DirectoriesToAlwaysCook=(Path="/Game/ACESim/Environment")' in default_game
    assert '+DirectoriesToAlwaysStageAsNonUFS=(Path="ACESim/Environment/Airport")' in default_game
    assert '+DirectoriesToAlwaysStageAsNonUFS=(Path="ACESim/Environment/Heliport")' in default_game
    assert '+DirectoriesToAlwaysCook=(Path="/Engine/EngineMaterials")' not in default_game


def test_ue5_orbit_camera_allows_close_inspection_zoom() -> None:
    project_root = ACESIMUE_ROOT

    free_camera_header = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.h").read_text(encoding="utf-8")
    free_camera_cpp = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").read_text(encoding="utf-8")

    assert "MinOrbitDistanceCm = 35.0f" in free_camera_header
    assert "OrbitZoomStepCm" not in free_camera_header
    assert "OrbitZoomScalePerWheelStep = 0.86f" in free_camera_header
    assert "FMath::Pow(OrbitZoomScalePerWheelStep" in free_camera_cpp
    assert (
        "OrbitDistanceCm = FMath::Clamp(OrbitDistanceCm * ZoomScale, MinOrbitDistanceCm, MaxOrbitDistanceCm)"
        in free_camera_cpp
    )
    assert "OrbitDistanceCm - WheelDelta * OrbitZoomStepCm" not in free_camera_cpp


def test_ue5_project_frames_vehicle_on_helipad_anchor_and_logs_visibility() -> None:
    project_root = ACESIMUE_ROOT

    game_mode_cpp = (project_root / "Source" / "ACESimUE" / "ACESimUEGameMode.cpp").read_text(encoding="utf-8")
    free_camera_cpp = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").read_text(encoding="utf-8")

    assert "GetHelipadAnchorLocation" in game_mode_cpp
    assert "VehicleActor->SetActorLocation(HelipadAnchorLocation)" in game_mode_cpp
    assert "VehicleActor->SetWorldOriginOffsetCm(HelipadAnchorLocation)" in game_mode_cpp
    assert "FrameVehicleWithEnvironment" in game_mode_cpp
    assert "FrameVehicleShadowCheck" in game_mode_cpp
    assert "ACESim vehicle visible in visual smoke frame" in game_mode_cpp
    assert "FrameVehicleWithEnvironment(AActor* VehicleActor, const FVector& EnvironmentAnchor)" in free_camera_cpp
    assert "FrameVehicleShadowCheck(AActor* VehicleActor, const FVector& EnvironmentAnchor)" in free_camera_cpp
    assert "OrbitTarget = (Bounds.GetCenter() * 0.72f) + (EnvironmentAnchor * 0.28f)" in free_camera_cpp
    assert "OrbitDistanceCm = FMath::Clamp(FMath::Max(980.0f, RadiusCm * 3.8f)" in free_camera_cpp
    assert "OrbitDistanceCm = FMath::Clamp(FMath::Max(360.0f, RadiusCm * 2.1f)" in free_camera_cpp


def test_ue5_orbit_camera_pitch_is_inverted_like_mujoco() -> None:
    project_root = ACESIMUE_ROOT

    free_camera_cpp = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").read_text(encoding="utf-8")

    assert "OrbitPitchDegrees = FMath::Clamp(OrbitPitchDegrees - Delta.Y * OrbitRotateSensitivity" in free_camera_cpp
    assert (
        "OrbitPitchDegrees = FMath::Clamp(OrbitPitchDegrees - PitchInput * KeyboardLookRateDegPerSec" in free_camera_cpp
    )
    assert "OrbitPitchDegrees + Delta.Y * OrbitRotateSensitivity" not in free_camera_cpp
    assert "OrbitPitchDegrees + PitchInput * KeyboardLookRateDegPerSec" not in free_camera_cpp


def test_ue5_project_accepts_golden_hour_lighting_preset() -> None:
    project_root = ACESIMUE_ROOT

    field_cpp = (project_root / "Source" / "ACESimUE" / "ACESimOutdoorTestFieldActor.cpp").read_text(encoding="utf-8")
    assert "ResolveAcesimLightingPresetName" in field_cpp
    assert 'FParse::Value(FCommandLine::Get(), TEXT("ACESimLightingPreset="), PresetName)' in field_cpp
    assert 'const TCHAR* LightingPresetName = TEXT("golden_hour")' not in field_cpp
    assert 'FCString::Strcmp(*LightingPresetName, TEXT("golden_hour"))' in field_cpp


def test_ue5_project_accepts_mythic_forest_day_lighting_preset() -> None:
    project_root = ACESIMUE_ROOT

    field_cpp = (project_root / "Source" / "ACESimUE" / "ACESimOutdoorTestFieldActor.cpp").read_text(encoding="utf-8")
    assert "ResolveAcesimLightingPresetName" in field_cpp
    assert 'const TCHAR* LightingPresetName = TEXT("mythic_forest_day")' not in field_cpp
    assert 'FCString::Strcmp(*LightingPresetName, TEXT("mythic_forest_day"))' in field_cpp
    assert "SunLight->SetRelativeRotation(FRotator(52.0f, -42.0f, 0.0f))" in field_cpp
    assert "SunLight->SetIntensity(18.0f)" in field_cpp
    assert "SunLight->SetLightColor(FLinearColor(1.0f, 0.91f, 0.78f), true)" in field_cpp
    assert "SunLight->ContactShadowLength = 80.0f" in field_cpp
    assert "SkyLight->SetIntensity(1.35f)" in field_cpp
    assert "HeightFog->SetFogDensity(0.00018f)" in field_cpp
    assert "SkyAtmosphere->SetSkyLuminanceFactor(FLinearColor(1.85f, 1.90f, 2.05f, 1.0f))" in field_cpp
    assert "SkyAtmosphere->SetAerialPespectiveViewDistanceScale(0.90f)" in field_cpp
    assert "HeightFog->SetFogInscatteringColor(FLinearColor(0.70f, 0.72f, 0.68f, 1.0f))" in field_cpp
    assert "HeightFog->SetVolumetricFog(true)" in field_cpp
    assert "VolumetricCloud->SetVisibility(true)" in field_cpp
    assert "PostProcess->Settings.AutoExposureBias = 0.48f" in field_cpp
    assert "PostProcess->Settings.LocalExposureShadowContrastScale = 0.48f" in field_cpp
    assert "PostProcess->Settings.ColorSaturation = FVector4(1.04f, 1.04f, 0.92f, 1.0f)" in field_cpp
    assert "PostProcess->Settings.ColorContrast = FVector4(1.12f, 1.10f, 1.06f, 1.0f)" in field_cpp
    assert (
        "SkyDomeMaterialInstance->SetVectorParameterValue("
        'TEXT("Zenith Color"), FLinearColor(0.95f, 1.22f, 2.05f, 1.0f))' in field_cpp
    )
    assert (
        "SkyDomeMaterialInstance->SetVectorParameterValue("
        'TEXT("Horizon color"), FLinearColor(1.35f, 1.40f, 1.50f, 1.0f))' in field_cpp
    )
    assert (
        "SkyDomeMaterialInstance->SetVectorParameterValue("
        'TEXT("Overall Color"), FLinearColor(1.78f, 1.72f, 1.55f, 1.0f))' in field_cpp
    )
    assert (
        'SkyDomeMaterialInstance->SetVectorParameterValue(TEXT("Sun color"), FLinearColor(1.0f, 0.88f, 0.66f, 1.0f))'
        in field_cpp
    )
    assert 'SkyDomeMaterialInstance->SetScalarParameterValue(TEXT("Cloud opacity"), 0.08f)' in field_cpp
    assert "ACESim warm mythic forest daylight ready" in field_cpp


def test_ue5_vehicle_meshes_cast_dynamic_shadows() -> None:
    project_root = ACESIMUE_ROOT

    actor_cpp = (
        project_root / "Plugins" / "ACESimBridge" / "Source" / "ACESimBridge" / "Private" / "ACESimVehicleActor.cpp"
    ).read_text(encoding="utf-8")

    assert "MeshComponent->SetCastShadow(true)" in actor_cpp
    assert "MeshComponent->bCastDynamicShadow = true" in actor_cpp
    assert "MeshComponent->bCastContactShadow = true" in actor_cpp
    assert "BodyMeshComponent->SetCastShadow(true)" in actor_cpp
    assert "BodyMeshComponent->bCastDynamicShadow = true" in actor_cpp
    assert "BodyMeshComponent->bCastContactShadow = true" in actor_cpp


def test_ue5_project_uses_fixed_lumen_default_engine_config() -> None:
    default_engine = (ACESIMUE_ROOT / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")

    assert "+TargetedRHIs=SF_VULKAN_SM5" in default_engine
    assert "+TargetedRHIs=SF_VULKAN_SM6" not in default_engine
    assert "r.RayTracing=True" not in default_engine
    assert "r.Lumen.HardwareRayTracing=False" in default_engine
    assert "r.DynamicGlobalIlluminationMethod=1" in default_engine
    assert "r.ScreenPercentage=75" in default_engine
    assert "r.TemporalAA.Upsampling=True" in default_engine
    assert "r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange=1" not in default_engine
    assert "r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange=0" in default_engine
    assert "r.Lumen.FinalGather.Quality=2" in default_engine
    assert "r.SkyAtmosphere.FastSkyLUT=1" in default_engine
    assert "r.VolumetricCloud=1" in default_engine
    assert "r.VolumetricFog=1" in default_engine
    assert "sg.GlobalIlluminationQuality=2" in default_engine
    assert "sg.ReflectionQuality=2" in default_engine
    assert "sg.ShadowQuality=2" in default_engine
    assert "sg.EffectsQuality=3" in default_engine
