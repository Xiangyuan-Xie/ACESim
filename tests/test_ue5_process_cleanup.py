from __future__ import annotations

from unittest.mock import patch

from acesim.tools.ue5 import cleanup_ue_processes


def test_find_unreal_processes_keeps_only_current_user_and_targets() -> None:
    ps_output = "\n".join(
        [
            "101 1 xxy S 00:01 UnrealEditor /tmp/UnrealEditor project.uproject -game",
            "102 1 other S 00:01 UnrealEditor /tmp/UnrealEditor other.uproject",
            "103 1 xxy S 00:01 python3 python3 unrelated.py",
            "104 1 xxy S 00:01 zenserver /home/xxy/.config/Epic/zenserver --port 8558",
            "",
        ]
    )

    processes = cleanup_ue_processes.parse_ps_output(ps_output, current_user="xxy", current_pid=999)

    assert [process.pid for process in processes] == [101, 104]
    assert processes[0].command == "UnrealEditor"
    assert "zenserver" in processes[1].args


def test_find_unreal_processes_handles_truncated_comm_and_console_scripts() -> None:
    ps_output = "\n".join(
        [
            (
                "201 1 xxy S 00:01 UnrealTraceSer "
                "/tmp/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/UnrealTraceServer fork"
            ),
            "202 1 xxy S 00:01 python3 python3 /home/xxy/.local/bin/acesim_play_ue --ue-mode package",
            "203 1 xxy S 00:01 zsh zsh -lc pgrep UnrealTraceServer",
        ]
    )

    processes = cleanup_ue_processes.parse_ps_output(ps_output, current_user="xxy", current_pid=999)

    assert [process.pid for process in processes] == [201, 202]


def test_cleanup_dry_run_does_not_signal_processes() -> None:
    processes = [
        cleanup_ue_processes.ProcessInfo(
            pid=101,
            ppid=1,
            user="xxy",
            stat="S",
            elapsed="00:01",
            command="ACESimUE",
            args="/tmp/ACESimUE -Windowed",
        )
    ]

    with patch.object(cleanup_ue_processes.os, "kill") as kill:
        result = cleanup_ue_processes.cleanup_processes(processes, dry_run=True, wait_sec=0.0)

    assert result == {"terminated": [], "killed": []}
    kill.assert_not_called()


def test_cleanup_terms_then_kills_still_alive_process() -> None:
    processes = [
        cleanup_ue_processes.ProcessInfo(
            pid=101,
            ppid=1,
            user="xxy",
            stat="S",
            elapsed="00:01",
            command="UnrealTraceServer",
            args="UnrealTraceServer fork",
        )
    ]

    with patch.object(cleanup_ue_processes.os, "kill") as kill, patch.object(cleanup_ue_processes.time, "sleep"):
        result = cleanup_ue_processes.cleanup_processes(processes, dry_run=False, wait_sec=0.0)

    assert result == {"terminated": [101], "killed": [101]}
    assert kill.call_args_list[0].args == (101, cleanup_ue_processes.signal.SIGTERM)
    assert kill.call_args_list[1].args == (101, 0)
    assert kill.call_args_list[2].args == (101, cleanup_ue_processes.signal.SIGKILL)
