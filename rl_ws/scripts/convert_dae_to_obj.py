#!/usr/bin/env python3
"""
DAE（COLLADA）メッシュをOBJ形式に変換するスクリプト

MuJoCoはDAE形式をサポートしていないため、
OBJ形式への変換が必要。

Usage:
    cd rl_ws
    uv run python scripts/convert_dae_to_obj.py
"""

from pathlib import Path
import trimesh
import numpy as np


# Genesis DAEファイルのディレクトリ
GENESIS_DAE_DIR = Path("genesis_official/genesis/assets/urdf/go2/dae")

# 出力ディレクトリ
OUTPUT_DIR = Path("assets/go2_genesis_meshes")

# DAEファイルとパーツ名のマッピング
DAE_FILES = [
    "base.dae",
    "hip.dae",
    "thigh.dae",
    "thigh_mirror.dae",
    "calf.dae",
    "calf_mirror.dae",
    "foot.dae",
]


def convert_dae_to_obj(
    input_path: Path,
    output_path: Path,
    verbose: bool = True,
) -> bool:
    """
    DAEファイルをOBJに変換

    Args:
        input_path: 入力DAEファイルのパス
        output_path: 出力OBJファイルのパス
        verbose: 詳細出力を行うか

    Returns:
        変換成功フラグ
    """
    try:
        # trimeshでDAEを読み込み
        # force='scene'で複数メッシュをシーンとして読み込む
        scene_or_mesh = trimesh.load(str(input_path), force='scene')

        if isinstance(scene_or_mesh, trimesh.Scene):
            # 複数のメッシュを含むシーンの場合
            meshes = []
            for name, geom in scene_or_mesh.geometry.items():
                if isinstance(geom, trimesh.Trimesh):
                    meshes.append(geom)
                if verbose:
                    print(f"    - Found geometry: {name} "
                          f"(vertices: {len(geom.vertices) if isinstance(geom, trimesh.Trimesh) else 'N/A'})")

            if not meshes:
                print(f"  WARNING: No valid meshes found in {input_path}")
                return False

            # 全てのメッシュを結合
            combined = trimesh.util.concatenate(meshes)
        else:
            # 単一メッシュの場合
            combined = scene_or_mesh

        if verbose:
            print(f"  - Combined mesh: {len(combined.vertices)} vertices, "
                  f"{len(combined.faces)} faces")

        # OBJとして保存
        output_path.parent.mkdir(parents=True, exist_ok=True)
        combined.export(str(output_path), file_type='obj')

        if verbose:
            print(f"  - Saved: {output_path}")

        return True

    except Exception as e:
        print(f"  ERROR converting {input_path}: {e}")
        return False


def main():
    """メイン処理"""
    print("=== DAE to OBJ Conversion ===")

    # スクリプトの実行ディレクトリを基準にパスを解決
    script_dir = Path(__file__).parent.parent
    dae_dir = script_dir / GENESIS_DAE_DIR
    output_dir = script_dir / OUTPUT_DIR

    print(f"Input directory:  {dae_dir}")
    print(f"Output directory: {output_dir}")

    if not dae_dir.exists():
        print(f"ERROR: DAE directory not found: {dae_dir}")
        return

    # 出力ディレクトリを作成
    output_dir.mkdir(parents=True, exist_ok=True)

    # 変換結果のサマリー
    success_count = 0
    fail_count = 0

    for dae_file in DAE_FILES:
        input_path = dae_dir / dae_file
        output_path = output_dir / dae_file.replace(".dae", ".obj")

        print(f"\nConverting: {dae_file}")

        if not input_path.exists():
            print(f"  WARNING: File not found: {input_path}")
            fail_count += 1
            continue

        if convert_dae_to_obj(input_path, output_path):
            success_count += 1
        else:
            fail_count += 1

    print(f"\n=== Conversion Complete ===")
    print(f"Success: {success_count}, Failed: {fail_count}")


if __name__ == "__main__":
    main()
