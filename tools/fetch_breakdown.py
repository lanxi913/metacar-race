import json
import sys
import urllib.request

TOKEN = "0bffbfc6303848f804d5c2bbcf36007e66bf885e"
BASE = "https://api.i-mylab.com/api/"


def get(path):
    req = urllib.request.Request(BASE + path)
    req.add_header("Authorization", "Token " + TOKEN)
    req.add_header("User-Agent", "Mozilla/5.0")
    with urllib.request.urlopen(req, timeout=25) as r:
        return json.loads(r.read().decode("utf-8", "replace"))


scene = sys.argv[1] if len(sys.argv) > 1 else "lianxu4"
n = int(sys.argv[2]) if len(sys.argv) > 2 else 3
d = get(f"car/scoring/run-records/?page_size={n}&scene={scene}")
rows = d["results"]
out_path = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\latest_breakdown.md"

lines = []
for r in rows:
    det = get(f"car/scoring/run-records/{r['id']}/")
    lines.append(f"## run {r['id']}  {det['scene_slug']}  总分 {det['score']}  ({det['start_time'][:19]})")
    lines.append("")
    lines.append("| 元素 | 得分 | 扣分/加分原因 |")
    lines.append("|---|---|---|")
    for s in det.get("subscene_details") or []:
        rs = "<br>".join(f"{x['reason']} {x['score']}" for x in (s.get("reasons") or [])) or "—"
        lines.append(f"| {s['subscene_name']} | {s['score']} | {rs} |")
    lines.append("")

with open(out_path, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))
print("wrote", out_path)
