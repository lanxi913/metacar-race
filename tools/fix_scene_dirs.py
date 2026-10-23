# -*- coding: utf-8 -*-
"""修复解压后多一层目录的场景包。

有些场景 zip 里带一层同名目录（如 stop/stop/SmartCar.exe），客户端找不到
SmartCar.exe 就报「场景文件不存在」。这个脚本把多余的一层压平。

用法：
    .venv\\Scripts\\python.exe -u tools/fix_scene_dirs.py
"""
import pathlib
import shutil

ROOT = pathlib.Path(r"D:\元动力场景\scenes")


def flatten(scene_dir: pathlib.Path) -> bool:
    """如果目录下只有一个子目录且没有 SmartCar.exe，就把子目录内容提上来。"""
    entries = [p for p in scene_dir.iterdir()]
    if any(p.is_file() and p.name.lower() == "smartcar.exe" for p in entries):
        return False
    dirs = [p for p in entries if p.is_dir()]
    files = [p for p in entries if p.is_file()]
    if len(dirs) != 1 or files:
        return False

    inner = dirs[0]
    if not any(p.name.lower() == "smartcar.exe" for p in inner.iterdir()):
        return False

    tmp = scene_dir.parent / (scene_dir.name + "__tmp")
    if tmp.exists():
        shutil.rmtree(tmp)
    inner.rename(tmp)
    scene_dir.rmdir()
    tmp.rename(scene_dir)
    return True


def main() -> int:
    fixed = 0
    for d in sorted(ROOT.iterdir()):
        if not d.is_dir():
            continue
        if flatten(d):
            print(f"[fix] {d.name} 压平了一层目录")
            fixed += 1
        else:
            exe = d / "SmartCar.exe"
            print(f"[ok ] {d.name} {'有 SmartCar.exe' if exe.exists() else '结构未知'}")
    print(f"\n共修复 {fixed} 个")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
