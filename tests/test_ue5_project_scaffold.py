import json
from pathlib import Path

from acesim.tools.ue5 import create_project_scaffold


def _bridge_source_root(project_root: Path) -> Path:
    return project_root / "Plugins" / "ACESimBridge" / "Source" / "ACESimBridge"


def test_ue5_scaffold_generates_bridge_plugin(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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


def test_ue5_scaffold_links_zmq_and_preserves_wire_size(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

    source_root = _bridge_source_root(project_root)
    build_cs = (source_root / "ACESimBridge.Build.cs").read_text(encoding="utf-8")
    sync_cpp = (source_root / "Private" / "ACESimVehicleSyncComponent.cpp").read_text(encoding="utf-8")

    assert 'PublicSystemIncludePaths.Add("/usr/include");' in build_cs
    assert 'PublicSystemLibraryPaths.Add("/usr/lib/x86_64-linux-gnu");' in build_cs
    assert 'PublicSystemLibraries.Add("zmq");' in build_cs
    assert "static_assert(sizeof(FACESimVisualWireSample) == 196" in sync_cpp


def test_ue5_scaffold_reserves_disabled_sensor_feedback(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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


def test_ue5_vehicle_sync_uses_latest_sample_and_timeout(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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


def test_ue5_vehicle_actor_is_visible_and_self_syncing(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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
    assert 'CreateDefaultSubobject<UACESimVehicleSyncComponent>(TEXT("ACESimVehicleSync"))' in actor_cpp
    assert 'CreateDefaultSubobject<UACESimArmStateSyncComponent>(TEXT("ACESimArmStateSync"))' in actor_cpp
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


def test_ue5_scaffold_generates_arm_state_sync_component(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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
    assert "ACESim arm state stream connected" in arm_cpp
    assert "7-joint" in arm_cpp
    assert "5-joint" in arm_cpp


def test_ue5_project_uses_free_camera_without_default_pawn_sphere(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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
    assert "SetViewTarget(VehicleActor)" not in game_mode_cpp
    assert "SetViewTarget(*It)" not in game_mode_cpp
    assert "UFloatingPawnMovement" in free_camera_header
    assert "UCameraComponent" in free_camera_header
    assert "FrameVehicle(AActor* VehicleActor)" in free_camera_header
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


def test_ue5_free_camera_has_reliable_right_mouse_capture_and_keyboard_look_fallback(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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


def test_ue5_vehicle_actor_does_not_double_apply_manifest_root_body_transform(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

    actor_cpp = (_bridge_source_root(project_root) / "Private" / "ACESimVehicleActor.cpp").read_text(encoding="utf-8")

    assert 'GetStringField(*ManifestRootObject, TEXT("name"))' in actor_cpp
    assert "const bool bIsManifestRootBody = BodyName == ManifestRootBodyName" in actor_cpp
    assert "BodyComponent->SetRelativeLocation(FVector::ZeroVector)" in actor_cpp
    assert "BodyComponent->SetRelativeRotation(FRotator::ZeroRotator)" in actor_cpp


def test_ue5_project_disables_mouse_capture_on_launch(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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


def test_ue5_project_auto_spawns_vehicle_actor_on_play(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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
    assert "OpenWorld" not in default_engine
    assert "GlobalDefaultGameMode=/Script/ACESimUE.ACESimUEGameMode" in default_engine
    assert '+DirectoriesToAlwaysCook=(Path="/Game/ACESim/x500_arm2x")' in default_game


def test_ue5_project_generates_lightweight_outdoor_test_field(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

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
    assert "EnvironmentRootComponent" in field_header
    assert "EnvironmentSourceAttribution" in field_header
    assert "HelipadAnchorComponent" in field_header
    assert "PrimaryActorTick.bCanEverTick = false" in field_cpp
    assert "UInstancedStaticMeshComponent" not in field_header
    assert "UInstancedStaticMeshComponent" not in field_cpp
    assert "UDirectionalLightComponent" in field_header
    assert "USkyLightComponent" in field_header
    assert "UExponentialHeightFogComponent" in field_header
    assert "CreateDefaultSubobject<UDirectionalLightComponent>" in field_cpp
    assert "CreateDefaultSubobject<USkyLightComponent>" in field_cpp
    assert "CreateDefaultSubobject<UExponentialHeightFogComponent>" in field_cpp
    assert "SunLight->SetRelativeRotation(FRotator(-42.0f, -28.0f, 0.0f))" in field_cpp
    assert "SunLight->SetIntensity(5.2f)" in field_cpp
    assert "SkyLight->SetIntensity(1.6f)" in field_cpp
    assert "HeightFog->SetFogDensity(0.0018f)" in field_cpp
    assert "/Game/ACESim/Environment/Heliport/Model" in field_cpp
    assert "/Game/ACESim/Environment/Airport/Model" in field_cpp
    assert "heliport_manifest.json" in field_cpp
    assert "airport_manifest.json" in field_cpp
    assert "ACESim heliport assets loaded" in field_cpp
    assert "ACESim heliport attribution loaded" in field_cpp
    assert "ACESim heliport asset missing" in field_cpp
    assert "ACESim airport assets loaded" in field_cpp
    assert "ACESim airport attribution loaded" in field_cpp
    assert "ACESim airport asset missing" in field_cpp
    assert "FAssetData" in field_cpp
    assert "GetAssetsByPath" in field_cpp
    assert "EnvironmentRootComponent->SetWorldScale3D(FVector(100.0f))" in field_cpp
    assert "HelipadAnchorComponent->SetRelativeLocation(FVector::ZeroVector)" in field_cpp
    assert "GetHelipadAnchorLocation" in field_cpp
    assert "ACESim-controlled helipad anchor" in field_cpp
    assert "Component->SetCastShadow(true)" in field_cpp
    assert "Component->SetCollisionEnabled(ECollisionEnabled::QueryOnly)" in field_cpp
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
    assert "ACESim outdoor field asset missing" in field_cpp
    assert "AllAssetsLoaded" in field_cpp
    assert "UE_LOG(LogTemp, Error" in field_cpp
    assert "SpawnActor<AActor>" not in field_cpp
    assert '"Slate"' in build_cs
    assert '"AssetRegistry"' in build_cs
    assert '+DirectoriesToAlwaysCook=(Path="/Game/ACESim/Environment")' in default_game
    assert '+DirectoriesToAlwaysStageAsNonUFS=(Path="ACESim/Environment/Airport")' in default_game
    assert '+DirectoriesToAlwaysStageAsNonUFS=(Path="ACESim/Environment/Heliport")' in default_game
    assert '+DirectoriesToAlwaysCook=(Path="/Engine/EngineMaterials")' not in default_game


def test_ue5_project_frames_vehicle_on_helipad_anchor_and_logs_visibility(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

    game_mode_cpp = (project_root / "Source" / "ACESimUE" / "ACESimUEGameMode.cpp").read_text(encoding="utf-8")
    free_camera_cpp = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").read_text(encoding="utf-8")

    assert "GetHelipadAnchorLocation" in game_mode_cpp
    assert "VehicleActor->SetActorLocation(HelipadAnchorLocation)" in game_mode_cpp
    assert "FrameVehicleWithEnvironment" in game_mode_cpp
    assert "ACESim vehicle visible in visual smoke frame" in game_mode_cpp
    assert "FrameVehicleWithEnvironment(AActor* VehicleActor, const FVector& EnvironmentAnchor)" in free_camera_cpp
    assert "OrbitTarget = (Bounds.GetCenter() * 0.72f) + (EnvironmentAnchor * 0.28f)" in free_camera_cpp
    assert "OrbitDistanceCm = FMath::Clamp(FMath::Max(980.0f, RadiusCm * 3.8f)" in free_camera_cpp


def test_ue5_orbit_camera_pitch_is_inverted_like_mujoco(tmp_path: Path) -> None:
    project_root = tmp_path / "ACESimUE"

    create_project_scaffold.generate_project(project_root, overwrite=False)

    free_camera_cpp = (project_root / "Source" / "ACESimUE" / "ACESimFreeCameraPawn.cpp").read_text(encoding="utf-8")

    assert "OrbitPitchDegrees = FMath::Clamp(OrbitPitchDegrees - Delta.Y * OrbitRotateSensitivity" in free_camera_cpp
    assert (
        "OrbitPitchDegrees = FMath::Clamp(OrbitPitchDegrees - PitchInput * KeyboardLookRateDegPerSec" in free_camera_cpp
    )
    assert "OrbitPitchDegrees + Delta.Y * OrbitRotateSensitivity" not in free_camera_cpp
    assert "OrbitPitchDegrees + PitchInput * KeyboardLookRateDegPerSec" not in free_camera_cpp


def test_ue5_project_render_presets_control_lumen_and_raytracing(tmp_path: Path) -> None:
    performance_root = tmp_path / "performance"
    raytracing_root = tmp_path / "raytracing"

    create_project_scaffold.generate_project(performance_root, overwrite=False, render_preset="performance")
    create_project_scaffold.generate_project(raytracing_root, overwrite=False, render_preset="raytracing")

    performance_engine = (performance_root / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")
    raytracing_engine = (raytracing_root / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")

    assert "+TargetedRHIs=SF_VULKAN_SM5" in performance_engine
    assert "+TargetedRHIs=SF_VULKAN_SM6" not in performance_engine
    assert "r.RayTracing=True" not in performance_engine
    assert "r.Lumen.HardwareRayTracing=True" not in performance_engine
    assert "r.DynamicGlobalIlluminationMethod=0" in performance_engine
    assert "+TargetedRHIs=SF_VULKAN_SM6" in raytracing_engine
    assert "+TargetedRHIs=SF_VULKAN_SM5" not in raytracing_engine
    assert "r.RayTracing=True" in raytracing_engine
    assert "r.Lumen.HardwareRayTracing=True" in raytracing_engine
    assert "r.DynamicGlobalIlluminationMethod=1" in raytracing_engine
