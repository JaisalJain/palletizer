"""Assemble renders/hero2/shot_XX_*.mp4 into the README hero video.
    python make_hero_video2.py <hero2_dir> <out_dir>
-> palletizer_hero.mp4 (GitHub-ready, < 10 MB), palletizer_hero_master.mp4 (CRF 18), poster jpg."""
import glob, os, subprocess, sys, json
SRC, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
CAPTIONS = {
    "infeed": "Cartons arrive at random ±25°  ·  sizes are measured, never configured",
    "pick": "Suction pick at the measured top-face centre  ·  weighed on the F/T sensor",
    "place": "Level transit via the +x swing  ·  vertical descent onto the perceived stack",
    "layer": "Pattern complete  ·  21 boxes, 3 layers × 7",
    "out": "Pallet exchange  ·  full pallet out through the light curtain",
    "in": "New pallet in  ·  its pose and size are measured, not assumed",
    "first": "…and the next pallet begins",
}
shots = sorted(glob.glob(os.path.join(SRC, "shot_*.mp4")))
def dur(p):
    return float(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", p]))
t, spans = 0.0, []
for p in shots:
    d = dur(p); name = os.path.basename(p)[8:-4]; spans.append((name, t, t + d)); t += d
TOTAL = t
lst = os.path.join(OUT, "_concat.txt")
open(lst, "w").write("".join(f"file '{os.path.abspath(p)}'\n" for p in shots))

def esc(s): return s.replace(":", r"\:").replace("'", r"’")
vf = ["fade=t=in:st=0:d=0.6"]
for name, a, b in spans:
    if name not in CAPTIONS: continue
    a1, b1 = a + 0.35, b - 0.25
    al = f"if(lt(t,{a1}),0,if(lt(t,{a1+0.4}),(t-{a1})/0.4,if(gt(t,{b1-0.35}),max(0,({b1}-t)/0.35),1)))"
    vf.append(f"drawtext=fontfile={REG}:text='{esc(CAPTIONS[name])}':fontsize=h*0.030:fontcolor=white:"
              f"alpha='{al}':enable='between(t,{a1},{b1})':x=w*0.045:y=h*0.885:box=1:boxcolor=black@0.45:boxborderw=18")
T0 = spans[-1][1] + 0.4
at = f"if(lt(t,{T0}),0,if(lt(t,{T0}+0.7),(t-{T0})/0.7,1))"
vf.append(f"drawtext=fontfile={BOLD}:text='PALLETIZER':fontsize=h*0.11:fontcolor=white:alpha='{at}':x=(w-tw)/2:y=h*0.60:"
          f"box=1:boxcolor=black@0.38:boxborderw=28|60")
vf.append(f"drawtext=fontfile={REG}:text='Autonomous palletizing  ·  ROS 2  ·  Gazebo  ·  MoveIt 2  ·  UR10e':fontsize=h*0.033:"
          f"fontcolor=white:alpha='{at}':x=(w-tw)/2:y=h*0.78:shadowcolor=black@0.7:shadowx=2:shadowy=2")
vf.append(f"fade=t=out:st={TOTAL-0.7}:d=0.7,format=yuv420p")
VF = ",".join(vf)
base = ["ffmpeg", "-y", "-v", "error", "-f", "concat", "-safe", "0", "-i", lst, "-vf", VF, "-an", "-c:v", "libx264", "-preset", "slow"]
master = os.path.join(OUT, "palletizer_hero_master.mp4")
subprocess.check_call(base + ["-crf", "18", "-movflags", "+faststart", master])
kbps = int(9.2 * 8 * 1000 / TOTAL)
gh = os.path.join(OUT, "palletizer_hero.mp4")
for p in (1, 2):
    subprocess.check_call(["ffmpeg", "-y", "-v", "error", "-i", master, "-c:v", "libx264", "-preset", "slow", "-b:v", f"{kbps}k",
                           "-pass", str(p), "-passlogfile", "/tmp/h2p", "-an", "-movflags", "+faststart"] +
                          (["-f", "mp4", "/dev/null"] if p == 1 else [gh]))
pt = next(a for n, a, b in spans if n == "place") + 1.6
subprocess.check_call(["ffmpeg", "-y", "-v", "error", "-ss", str(pt), "-i", master, "-frames:v", "1", "-q:v", "2",
                       os.path.join(OUT, "palletizer_hero_poster.jpg")])
print(json.dumps({"total_s": round(TOTAL, 2), "shots": [(n, round(a, 2), round(b, 2)) for n, a, b in spans],
                  "github_mb": round(os.path.getsize(gh) / 1e6, 2), "master_mb": round(os.path.getsize(master) / 1e6, 2)}, indent=1))
