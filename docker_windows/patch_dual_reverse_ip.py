#!/usr/bin/env python3
"""Patch only the installed Dual-UR7e launch copy for Docker reverse networking.

The source repository is deliberately not edited.  The current ur7e_tools
custom ur_control_namespaced.launch.py already supports reverse_ip, while the
current dual_ur7e.launch.py does not forward it.  In Docker Desktop the robot
must be told to connect back to the Windows host NIC, not to the container's
private address.
"""

from pathlib import Path
import sys


def fail(msg: str) -> None:
    print(f"[reverse-ip patch] ERROR: {msg}", file=sys.stderr)
    raise SystemExit(2)


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: patch_dual_reverse_ip.py <installed dual_ur7e.launch.py>")

    path = Path(sys.argv[1])
    if not path.is_file():
        fail(f"launch file not found: {path}")

    text = path.read_text()

    # If this future branch already forwards reverse_ip, leave it untouched.
    if (
        'reverse_ip = LaunchConfiguration("reverse_ip")' in text
        and '"reverse_ip": reverse_ip' in text
    ):
        print("[reverse-ip patch] launch file already supports reverse_ip; no patch needed")
        return

    anchor = '    robot2_ip = LaunchConfiguration("robot2_ip")\n'
    if anchor not in text:
        fail("could not locate robot2_ip LaunchConfiguration anchor")
    text = text.replace(
        anchor,
        anchor + '    reverse_ip = LaunchConfiguration("reverse_ip")\n',
        1,
    )

    r1 = '                    "robot_ip": robot1_ip,\n'
    r2 = '                    "robot_ip": robot2_ip,\n'
    if r1 not in text or r2 not in text:
        fail("could not locate robot_ip mappings")
    text = text.replace(
        r1,
        r1 + '                    "reverse_ip": reverse_ip,\n',
        1,
    )
    text = text.replace(
        r2,
        r2 + '                    "reverse_ip": reverse_ip,\n',
        1,
    )

    declaration = '        DeclareLaunchArgument("robot2_ip", default_value="127.0.0.1"),\n'
    if declaration not in text:
        fail("could not locate robot2_ip declaration")
    text = text.replace(
        declaration,
        declaration
        + '        DeclareLaunchArgument(\n'
        + '            "reverse_ip",\n'
        + '            default_value=os.environ.get("UR_REVERSE_IP", "0.0.0.0"),\n'
        + '            description="Windows host IP used by the UR controllers for reverse connections.",\n'
        + '        ),\n',
        1,
    )

    path.write_text(text)
    print(f"[reverse-ip patch] patched installed launch copy: {path}")


if __name__ == "__main__":
    main()
