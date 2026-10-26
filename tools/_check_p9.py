"""离线复核 p9 的两处扣分：元素 3 的实线余量、元素 8 的停车中心距。"""
import csv
import sys

sys.path.insert(0, r'D:\无人算法\metacar-race')

from metacar_race.scene4 import borders  # noqa: E402

MAPD = r'D:\无人算法\场景包\scenes\lianxu4\SmartCar_Data\StreamingAssets\mapfile\mapd'


def main(path: str) -> None:
    rows = list(csv.DictReader(open(path, encoding='utf-8')))
    segs = borders.load_solid_segments(MAPD)
    grid = borders.BorderGrid(segs)
    print('solid segs', len(segs))

    for elem in ('3', '7', '9', '12'):
        sub = [r for r in rows if r['elem_i'] == elem]
        if not sub:
            continue
        worst = (9e9, None)
        for r in sub:
            try:
                c = grid.clearance(float(r['x']), float(r['y']), float(r['yaw']), 4.70, 1.90)
            except Exception:  # noqa: BLE001
                continue
            d = c[0] if isinstance(c, tuple) else c
            if d < worst[0]:
                worst = (d, r)
        r = worst[1]
        print('elem %-3s frames %5d min clearance %.3f m at t=%s (x=%s y=%s yaw=%s)' % (
            elem, len(sub), worst[0], r['sim_t'], r['x'], r['y'], r['yaw']))

    # 元素 8：blk_dist 就是车到前车的 route 投影距离；再看停车后速度
    sub = [r for r in rows if r['elem_i'] == '8']
    if sub:
        stopped = [r for r in sub if abs(float(r['speed'])) < 0.05]
        if stopped:
            r = stopped[0]
            print('elem 8 first stopped: t=%s blk_dist=%s lat_err=%s v=%s' % (
                r['sim_t'], r['blk_dist'], r['lat_err'], r['speed']))
        print('elem 8 last: t=%s blk_dist=%s' % (sub[-1]['sim_t'], sub[-1]['blk_dist']))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else r'logs\scene4\p9\run.csv')
