# -*- coding: utf-8 -*-
"""港口迷雾：把 static / task / task_state 冻结成文件，供离线做"别撞墙"的安全检查。

之后所有路径安全检查（离墙距离、障碍膨胀、替代路线）都不需要再开场景，
直接用这份 static.json 复算，省掉每趟 10 分钟的跑车。

用法：python -u fog_dump_static.py --out ...\\logs\\static1
"""
import argparse
import json
import os

from fog_map import connect_retry, to_plain


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    api = connect_retry()
    try:
        static = to_plain(api.get_scene_static_data())
        task = to_plain(api.get_task())
        ts = to_plain(api.get_task_state())
        for name, obj in (("static.json", static), ("task.json", task),
                          ("task_state.json", ts)):
            path = os.path.join(args.out, name)
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(obj, fh, ensure_ascii=False, indent=1)
            print("wrote %s" % path, flush=True)
        cats = {}
        for o in static.get("static_objects") or []:
            cats[o.get("category")] = cats.get(o.get("category"), 0) + 1
        print("static_objects=%d roads=%d" % (len(static.get("static_objects") or []),
                                              len(static.get("roads") or [])), flush=True)
        for k, v in sorted(cats.items(), key=lambda kv: -kv[1]):
            print("  %-24s %d" % (k, v), flush=True)
        print("vehicles:", [v.get("vehicle_id") for v in static.get("vehicles") or []], flush=True)
    finally:
        try:
            api.close()
        except Exception:                                      # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
