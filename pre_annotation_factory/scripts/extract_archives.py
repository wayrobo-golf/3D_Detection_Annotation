#!/usr/bin/env python3

from __future__ import annotations

import os
import stat
import shutil
import tarfile
import zipfile
from pathlib import Path
from typing import Iterable

# ====== 配置区：按需修改这两个目录 ======
SOURCE_DIR = Path(
    os.environ.get(
        "SOURCE_DIR",
        "/home/keyaoli/Data/AutoAnnotation/Wayrobo_Rosbag/YinXiu20260414/20260414",
    )
)
OUTPUT_DIR = Path(
    os.environ.get(
        "OUTPUT_DIR",
        "/home/keyaoli/Data/AutoAnnotation/Wayrobo_Rosbag/YinXiu20260414/20260414/extracted",
    )
)

# 可选配置
RECURSIVE = os.environ.get("RECURSIVE", "false").lower() == "true"
OVERWRITE = os.environ.get("OVERWRITE", "false").lower() == "true"
REMOVE_ARCHIVE = os.environ.get("REMOVE_ARCHIVE", "false").lower() == "true"

ARCHIVE_PATTERNS = (
    "*.tar.gz",
    "*.tgz",
    "*.tar",
    "*.tar.bz2",
    "*.tbz2",
    "*.tar.xz",
    "*.txz",
    "*.zip",
)


def log(level: str, message: str) -> None:
    print(f"[{level}] {message}")


def archive_type(archive: Path) -> str:
    name = archive.name.lower()
    if name.endswith((".tar.gz", ".tgz", ".tar", ".tar.bz2", ".tbz2", ".tar.xz", ".txz")):
        return "tar"
    if name.endswith(".zip"):
        return "zip"
    return "unknown"


def normalize_relative_path(path_str: str) -> Path | None:
    candidate = Path(path_str.strip())
    parts = []
    for part in candidate.parts:
        if part in ("", "."):
            continue
        if part == "..":
            return None
        parts.append(part)

    if not parts:
        return None

    normalized = Path(*parts)
    if normalized.is_absolute():
        return None
    return normalized


def top_level_entry(archive: Path, kind: str) -> str | None:
    try:
        if kind == "tar":
            with tarfile.open(archive, "r:*") as tar:
                for member in tar.getmembers():
                    normalized = normalize_relative_path(member.name)
                    if normalized is not None:
                        return normalized.parts[0]
        elif kind == "zip":
            with zipfile.ZipFile(archive) as zf:
                for name in zf.namelist():
                    normalized = normalize_relative_path(name)
                    if normalized is not None:
                        return normalized.parts[0]
    except (tarfile.TarError, zipfile.BadZipFile, OSError):
        return None
    return None


def safe_tar_members(tar: tarfile.TarFile) -> Iterable[tarfile.TarInfo]:
    for member in tar.getmembers():
        normalized = normalize_relative_path(member.name)
        if normalized is None:
            raise ValueError(f"tar 成员路径非法: {member.name}")
        if member.issym() or member.islnk():
            raise ValueError(f"tar 包含链接文件，已拒绝解压: {member.name}")
        member.name = normalized.as_posix()
        yield member


def extract_tar(archive: Path, output_dir: Path) -> None:
    with tarfile.open(archive, "r:*") as tar:
        tar.extractall(output_dir, members=safe_tar_members(tar))


def safe_zip_members(zf: zipfile.ZipFile) -> Iterable[zipfile.ZipInfo]:
    for info in zf.infolist():
        normalized = normalize_relative_path(info.filename)
        if normalized is None:
            raise ValueError(f"zip 成员路径非法: {info.filename}")
        mode = info.external_attr >> 16
        if stat.S_ISLNK(mode):
            raise ValueError(f"zip 包含链接文件，已拒绝解压: {info.filename}")
        info.filename = normalized.as_posix()
        yield info


def extract_zip(archive: Path, output_dir: Path) -> None:
    with zipfile.ZipFile(archive) as zf:
        zf.extractall(output_dir, members=safe_zip_members(zf))


def remove_existing_target(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path)
    elif path.exists():
        path.unlink()


def extract_archive(archive: Path, kind: str, output_dir: Path, overwrite: bool) -> None:
    target_name = top_level_entry(archive, kind)
    if target_name and overwrite:
        remove_existing_target(output_dir / target_name)

    if kind == "tar":
        extract_tar(archive, output_dir)
    elif kind == "zip":
        extract_zip(archive, output_dir)
    else:
        raise ValueError(f"不支持的压缩包类型: {archive.name}")


def find_archives(source_dir: Path, recursive: bool) -> list[Path]:
    archives: set[Path] = set()
    for pattern in ARCHIVE_PATTERNS:
        matches = source_dir.rglob(pattern) if recursive else source_dir.glob(pattern)
        archives.update(path for path in matches if path.is_file())
    return sorted(archives)


def main() -> int:
    if not SOURCE_DIR.is_dir():
        log("ERROR", f"SOURCE_DIR 不存在: {SOURCE_DIR}")
        return 1

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    archives = find_archives(SOURCE_DIR, RECURSIVE)

    if not archives:
        log("INFO", f"未找到压缩包。SOURCE_DIR={SOURCE_DIR}")
        return 0

    success_count = 0
    skip_count = 0
    fail_count = 0

    for archive in archives:
        kind = archive_type(archive)
        archive_name = archive.name

        if kind == "unknown":
            log("WARN", f"跳过不支持的文件: {archive_name}")
            skip_count += 1
            continue

        target_name = top_level_entry(archive, kind)
        if target_name and not OVERWRITE and (OUTPUT_DIR / target_name).exists():
            log("SKIP", f"{archive_name} -> {OUTPUT_DIR / target_name} 已存在")
            skip_count += 1
            continue

        log("INFO", f"开始解压: {archive_name}")
        try:
            extract_archive(archive, kind, OUTPUT_DIR, OVERWRITE)
            log("OK", f"解压完成: {archive_name}")
            success_count += 1

            if REMOVE_ARCHIVE:
                archive.unlink()
                log("INFO", f"已删除原压缩包: {archive_name}")
        except (OSError, ValueError, tarfile.TarError, zipfile.BadZipFile, RuntimeError) as exc:
            log("ERROR", f"解压失败: {archive_name} ({exc})")
            fail_count += 1

    print()
    log("INFO", "处理完成")
    log("INFO", f"成功: {success_count}")
    log("INFO", f"跳过: {skip_count}")
    log("INFO", f"失败: {fail_count}")
    log("INFO", f"输出目录: {OUTPUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
