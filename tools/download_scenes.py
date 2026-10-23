# -*- coding: utf-8 -*-
"""批量下载全部场景包（走 CDN 直链，跳过客户端下载确认框）。

用法：
    .venv\\Scripts\\python.exe -u tools/download_scenes.py [--only 关键字]

会做的事：
  1. 从 api.i-mylab.com 拉场景列表（39 个）
  2. 对每个还没装过的场景：下载 zip → 解压到 D:\\元动力场景\\scenes\\<slug>
  3. 更新 downloaded-scenes.json，客户端重启后即显示「启动场景」
"""
import argparse
import json
import os
import pathlib
import sys
import time
import urllib.request
import zipfile

SCENE_ROOT = pathlib.Path(r"D:\元动力场景\scenes")
MANIFEST = pathlib.Path(os.path.expandvars(
    r"%APPDATA%\com.i-mylab.metapower-desktop\car\downloaded-scenes.json"))
API = "https://api.i-mylab.com/api/scenes/"
UA = {"User-Agent": "Mozilla/5.0", "Referer": "https://www.i-mylab.com/icv/"}


def get_json(url):
    req = urllib.request.Request(url, headers=UA)
    raw = urllib.request.urlopen(req, timeout=60).read()
    try:
        return json.loads(raw.decode("utf-8"))
    except Exception:
        return json.loads(raw.decode("gbk"))


def load_manifest():
    if MANIFEST.exists():
        return json.loads(MANIFEST.read_text(encoding="utf-8"))
    return {}


def save_manifest(mf):
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(mf, ensure_ascii=False, indent=2), encoding="utf-8")


def folder_size(p: pathlib.Path) -> int:
    return sum(f.stat().st_size for f in p.rglob("*") if f.is_file())


def flatten(scene_dir: pathlib.Path) -> bool:
    """有些 zip 会多包一层同名目录（stop/stop/SmartCar.exe），压平它。"""
    import shutil

    entries = list(scene_dir.iterdir())
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None, help="只下名字里包含该关键字的场景")
    args = ap.parse_args()

    print("拉取场景列表...")
    data = get_json(API + "?page=1&page_size=100&category=")
    scenes = data["results"]
    print(f"共 {len(scenes)} 个场景")

    mf = load_manifest()
    ok = skip = fail = 0

    for sc in scenes:
        slug = sc["slug"]
        if args.only and args.only not in slug:
            continue

        dest = SCENE_ROOT / slug
        if slug in mf and dest.exists() and any(dest.iterdir()):
            print(f"[skip] {slug:<28} 已安装 {folder_size(dest) / 1e6:.0f} MB")
            skip += 1
            continue

        try:
            info = get_json(f"https://api.i-mylab.com/api/scenes/{slug}/download-info")
            url, version = info["url"], info.get("version")
            size_mb = info.get("size", 0) / 1e6
            print(f"\n[get ] {slug:<28} v{version}  {size_mb:.0f} MB")
        except Exception as exc:  # noqa: BLE001
            print(f"[err ] {slug} 取下载信息失败: {exc}")
            fail += 1
            continue

        tmp = pathlib.Path(os.environ.get("TEMP", ".")) / f"{slug}.zip"
        t0 = time.time()
        try:
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=600) as resp, open(tmp, "wb") as fh:
                while True:
                    chunk = resp.read(1 << 20)
                    if not chunk:
                        break
                    fh.write(chunk)
            dt = time.time() - t0
            print(f"       下载完成 {tmp.stat().st_size / 1e6:.0f} MB / {dt:.0f}s")
        except Exception as exc:  # noqa: BLE001
            print(f"[err ] {slug} 下载失败: {exc}")
            fail += 1
            continue

        try:
            dest.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(tmp) as zf:
                zf.extractall(dest)
            flatten(dest)
            print(f"       解压完成 {folder_size(dest) / 1e6:.0f} MB")
        except Exception as exc:  # noqa: BLE001
            print(f"[err ] {slug} 解压失败: {exc}")
            fail += 1
            continue
        finally:
            tmp.unlink(missing_ok=True)

        mf[slug] = {
            "name": sc.get("name", slug),
            "scene": slug,
            "size": folder_size(dest),
            "version": version,
        }
        save_manifest(mf)
        ok += 1

    print(f"\n完成：新增 {ok}，跳过 {skip}，失败 {fail}")
    print("客户端需要重启才能看到新场景")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
