#!/usr/bin/env python3
"""URDF Export Script

Exports URDF from ROS 2 workspace xacro files.
This creates a standalone URDF file that can be used with Genesis/MuJoCo.

Usage:
    # From rl_ws directory
    uv run python assets/export_urdf.py
    uv run python assets/export_urdf.py --model bsl_droid_simplified_v2

    # Or from repository root
    cd rl_ws && uv run python assets/export_urdf.py

Note:
    This script requires xacro to be installed. If running outside ROS 2,
    install with: pip install xacro
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


MODEL_MAP = {
    "biped_digitigrade": "biped_digitigrade.urdf.xacro",
    "bsl_droid_simplified": "bsl_droid_simplified.urdf.xacro",
    "bsl_droid_simplified_v2": "bsl_droid_simplified_v2.urdf.xacro",
}


def find_xacro_file(model: str) -> Path:
    """Find the xacro file for the given model.

    Args:
        model: Model name (key in MODEL_MAP)

    Returns:
        Path to xacro file

    Raises:
        FileNotFoundError: If xacro file not found
        ValueError: If model name is unknown
    """
    if model not in MODEL_MAP:
        raise ValueError(f"Unknown model: {model}\nAvailable models: {', '.join(MODEL_MAP.keys())}")

    script_dir = Path(__file__).parent
    repo_root = script_dir.parent.parent

    xacro_path = repo_root / "ros2_ws" / "src" / "biped_description" / "urdf" / MODEL_MAP[model]

    if xacro_path.exists():
        return xacro_path

    raise FileNotFoundError(
        f"URDF xacro file not found at: {xacro_path}\nMake sure you're running this from the rl_ws directory."
    )


def export_urdf(model: str, output_path: Path | None = None) -> Path:
    """Export URDF from xacro file.

    Args:
        model: Model name (key in MODEL_MAP)
        output_path: Optional output path. Defaults to assets/{model}.urdf

    Returns:
        Path to exported URDF file
    """
    xacro_path = find_xacro_file(model)

    if output_path is None:
        output_path = Path(__file__).parent / f"{model}.urdf"

    print(f"Exporting URDF from: {xacro_path}")
    print(f"Output: {output_path}")

    try:
        result = subprocess.run(
            ["xacro", str(xacro_path)],
            capture_output=True,
            text=True,
            check=True,
        )

        output_path.write_text(result.stdout)
        print(f"URDF exported successfully to: {output_path}")
        return output_path

    except FileNotFoundError:
        print("xacro command not found.")
        print("Install with: pip install xacro")
        print("Or use pixi from ros2_ws: pixi run xacro ...")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Error running xacro: {e.stderr}")
        sys.exit(1)


def verify_urdf(urdf_path: Path) -> bool:
    """Verify the exported URDF is valid.

    Args:
        urdf_path: Path to URDF file

    Returns:
        True if valid, False otherwise
    """
    try:
        import xml.etree.ElementTree as ET

        tree = ET.parse(urdf_path)
        root = tree.getroot()

        if root.tag != "robot":
            print("Warning: Root element is not 'robot'")
            return False

        links = root.findall("link")
        joints = root.findall("joint")

        print(f"URDF contains {len(links)} links and {len(joints)} joints")
        return True

    except Exception as e:
        print(f"URDF validation failed: {e}")
        return False


def main() -> None:
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Export biped URDF from xacro")
    parser.add_argument(
        "--model",
        type=str,
        default="biped_digitigrade",
        choices=list(MODEL_MAP.keys()),
        help="Model to export (default: biped_digitigrade)",
    )
    parser.add_argument("-o", "--output", type=Path, default=None, help="Output path for URDF file")
    parser.add_argument("--no-verify", action="store_true", help="Skip URDF verification")
    args = parser.parse_args()

    urdf_path = export_urdf(args.model, args.output)

    if not args.no_verify:
        verify_urdf(urdf_path)


if __name__ == "__main__":
    main()
