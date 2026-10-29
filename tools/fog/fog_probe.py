# -*- coding: utf-8 -*-
"""港口迷雾协同运输场景：一次性探针。

只做"看见什么"的盘点，不控制车辆：
  1. 静态数据（道路/障碍/区域/边界）
  2. 车辆清单与初始状态
  3. 任务定义与任务状态（信号进度、路障、路口、泊位）
  4. 四辆车各自的有效观测（可见对象、精度等级、观测凭证）

用法：
  python -u fog_probe.py --seconds 40 --out D:\\无人算法\\metacar-race\\logs\\fog_p1
"""
import argparse
import json
import os
import time

from metacar_multiagent import ScenarioTaskAPI

FIELDS = [
    "object_id", "object_type", "position", "uncertainty", "precision_level",
    "distance", "observation_token", "state", "road_ids", "lane_ids",
    "radius_m", "valid_time", "raw",
]


def jdump(obj, name, out):
    try:
        data = obj if isinstance(obj, dict) else (
            obj.to_dict() if hasattr(obj, "to_dict") else obj.__dict__)
    except Exception:
        data = str(obj)
    path = os.path.join(out, name + ".json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2, default=str)
    return path


def brief(item):
    d = {}
    for k in FIELDS:
        v = getattr(item, k, None)
        if v is None:
            continue
        if isinstance(v, (list, tuple)) and len(v) > 6:
            v = list(v)[:6] + ["..."]
        d[k] = v
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    api = ScenarioTaskAPI()
    ok = False
    deadline = time.time() + 180.0
    while time.time() < deadline:
        try:
            api.connect()
            ok = True
            break
        except Exception as exc:
            print("[fog] connect retry: %s" % exc, flush=True)
            time.sleep(2.0)
    if not ok:
        raise SystemExit("cannot connect 127.0.0.1:9100")
    try:
        print("[fog] connected", flush=True)

        st = api.get_scene_static_data()
        jdump(st, "static", args.out)
        roads = list(getattr(st, "roads", []) or [])
        print("[fog] roads=%d obstacles=%d regions=%d vehicles=%d"
              % (len(roads), len(getattr(st, "obstacles", []) or []),
                 len(getattr(st, "regions", []) or []),
                 len(getattr(st, "vehicles", []) or [])), flush=True)
        for v in (getattr(st, "vehicles", []) or []):
            print("   veh", getattr(v, "vehicle_id", None) or getattr(v, "id", None),
                  getattr(v, "vehicle_type", None) or getattr(v, "type", None),
                  getattr(v, "position", None), flush=True)

        vids = api.get_vehicle_ids()
        print("[fog] vehicle_ids =", vids, flush=True)

        task = api.get_task()
        jdump(task, "task", args.out)
        print("[fog] task:", getattr(task, "task_id", None),
              getattr(task, "title", None) or getattr(task, "name", None), flush=True)

        t0 = time.time()
        last = 0.0
        while time.time() - t0 < args.seconds:
            ts = api.get_task_state()
            jdump(ts, "task_state_%03d" % int(time.time() - t0), args.out)
            print("[fog t=%5.1f] signals=%s/%s roadblock_active=%s owner=%r "
                  "objectives=%s" % (
                      time.time() - t0,
                      getattr(ts, "verified_signal_count", "?"),
                      getattr(ts, "total_signal_count", "?"),
                      getattr(ts, "roadblock_active", "?"),
                      getattr(ts, "intersection_owner", "?"),
                      len(getattr(ts, "objectives", []) or [])), flush=True)

            for sid in vids:
                try:
                    vs = api.get_vehicle_state(sid)
                    pos = getattr(vs, "pose", None) or getattr(vs, "position", None)
                    print("   %s pose=%s speed=%.2f" % (
                        sid, pos, float(getattr(vs, "speed", 0.0) or 0.0)), flush=True)
                except Exception as exc:
                    print("   %s state error %s" % (sid, exc), flush=True)
                try:
                    ob = api.get_observation(sid)
                    jdump(ob, "obs_%s_%03d" % (sid, int(time.time() - t0)), args.out)
                    objs = list(getattr(ob, "visible_objects", []) or [])
                    print("   %s visible=%d" % (sid, len(objs)), flush=True)
                    for it in objs:
                        print("      ", json.dumps(brief(it), ensure_ascii=False,
                                                   default=str), flush=True)
                except Exception as exc:
                    print("   %s obs error %s" % (sid, exc), flush=True)

            time.sleep(max(0.0, 2.0 - (time.time() - t0 - last)))
            last = time.time() - t0

    finally:
        try:
            api.close()
        except Exception:
            pass
    print("[fog] done", flush=True)


if __name__ == "__main__":
    main()
