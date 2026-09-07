#!/usr/bin/env python3
"""Top-down trajectory viewer for judging the controller across sims.

For each run (a dir with frames.jsonl [+ optional result.json/config.json],
or a frames.jsonl path), draws the drone's x-y path over the obstacle
map, COLOURED BY CONTROLLER MODE (cruise/avoid/blind) so you can see
WHEN it reacted, with the true obstacle footprints, the collision ring,
start/end/collision markers, the goal line, and the minimum clearance.

The point is to judge the SHAPE of the avoidance -- smooth arc vs. late
jerk vs. oscillation vs. straight-in -- and its consistency across runs,
which raw collision-rate numbers hide. OFFLINE, saves a PNG.

Usage:
  tools/plot_trajectories.py RUN [RUN ...] --out traj.png [--overlay|--grid]
    RUN = a run directory (uses its frames.jsonl) or a *_frames.jsonl file.
  tools/plot_trajectories.py 'eval_results/heavy_cr_run*' --out cr.png --overlay
"""
import argparse
import glob
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np

from obst_avoidance.runtime.orchestrator import OBSTACLES, COLLISION_RADIUS_M

MODE_COLOR = {"cruise": "#2ca02c", "avoid": "#ff7f0e", "blind": "#d62728"}


def _resolve_frames(path):
    if os.path.isdir(path):
        f = os.path.join(path, "frames.jsonl")
        return f if os.path.exists(f) else None
    return path if path.endswith(".jsonl") and os.path.exists(path) else None


def _run_name(path):
    if os.path.isdir(path):
        return os.path.basename(path.rstrip("/"))
    return os.path.basename(path).replace("_frames.jsonl", "").replace(".jsonl", "")


def _load_result(path, frames_path):
    """Result matching (Codex review fix): for a <name>_frames.jsonl, the
    NAMED sibling <name>.json wins over a generic result.json in the same
    dir (which may belong to another run). For a run DIR, use its result.json.
    Bad/unreadable JSON -> {} (verdict shown as 'unknown', never guessed)."""
    if os.path.isdir(path):
        candidates = [os.path.join(path, "result.json")]
    else:
        candidates = ([frames_path[:-len("_frames.jsonl")] + ".json"]
                      if frames_path.endswith("_frames.jsonl") else [])
        candidates.append(os.path.join(os.path.dirname(frames_path), "result.json"))
    for fp in candidates:
        if os.path.exists(fp):
            try:
                d = json.load(open(fp))
                return d.get("result", d)
            except Exception:
                return {"_parse_error": fp}
    return {}


def _fnum(v):
    """None/missing -> nan (so plots break the line instead of drawing a lie)."""
    return float(v) if isinstance(v, (int, float)) else float("nan")


def load_run(path):
    frames_path = _resolve_frames(path)
    if not frames_path:
        return None
    meta = {}
    xs, ys, modes, ts = [], [], [], []
    cmd_yaw, meas_yaw, heading_err, cross_track = [], [], [], []
    for line in open(frames_path):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        if "meta" in r and "seq" not in r:   # metadata header, not a frame
            meta = r["meta"]
            continue
        if "seq" not in r:                    # not a frame row -> skip, never treat as data
            continue
        p = r.get("vehicle_position")
        if p is None:                          # no usable position this frame -> skip the sample
            continue
        cmd = r.get("cmd") or {}
        xs.append(p[0]); ys.append(p[1])
        modes.append(cmd.get("mode", "unknown"))
        ts.append(_fnum(r.get("t_capture")))
        # signed commanded yaw rate (prefer explicit field, fall back to cmd)
        cmd_yaw.append(_fnum(r.get("commanded_yaw_rate", cmd.get("yaw_rate"))))
        meas_yaw.append(_fnum(r.get("measured_yaw_rate")))
        heading_err.append(_fnum(r.get("heading_error")))
        cross_track.append(_fnum(r.get("cross_track_error")))
    if len(xs) < 1:                            # no usable rows -> not a run (Codex: don't crash bounds())
        return None
    result = _load_result(path, frames_path)
    return dict(name=_run_name(path), xs=np.array(xs), ys=np.array(ys),
                modes=modes, ts=np.array(ts), yaw_rates=np.array(cmd_yaw),
                meas_yaw=np.array(meas_yaw), heading_err=np.array(heading_err),
                cross_track=np.array(cross_track), meta=meta, result=result)


def _status(result):
    """Precise run label from stop_reason (Codex review: distinguish
    completed / collided / timed-out / user-stopped, never guess)."""
    if not result:
        return "unknown"
    if result.get("_parse_error"):
        return "unknown(bad result.json)"
    sr = result.get("stop_reason")
    mapping = {"perception_unavailable": "perception_unavailable", "perception_timeout": "timed_out(perception)", "endpoint_reached": "endpoint_reached", "collision": "COLLIDED", "goal_reached": "goal_x_crossed",
               "user_stop": "user_stopped", "camera_timeout": "timed_out(camera)",
               "wall_timeout": "timed_out(wall)"}
    if sr in mapping:
        return mapping[sr]
    # older logs without stop_reason: fall back to the booleans
    if result.get("collided"):
        return "COLLIDED"
    if result.get("completed"):
        return "goal_x_crossed"
    return "unknown"


def min_clearance(xs, ys):
    """Smallest surface-distance to any obstacle along the path (m).
    <=0 means contact with the obstacle body; the collision ring adds
    COLLISION_RADIUS_M on top."""
    best = 1e9
    for x, y in zip(xs, ys):
        for o in OBSTACLES:
            if o["kind"] == "circle":
                d = math.hypot(x - o["x"], y - o["y"]) - o["radius"]
            else:
                dx = max(abs(x - o["x"]) - o["half_x"], 0.0)
                dy = max(abs(y - o["y"]) - o["half_y"], 0.0)
                d = math.hypot(dx, dy)
            best = min(best, d)
    return best


def clearance_series(xs, ys):
    """Per-point surface-distance to the NEAREST obstacle along the path."""
    out = []
    for x, y in zip(xs, ys):
        best = 1e9
        for o in OBSTACLES:
            if o["kind"] == "circle":
                d = math.hypot(x - o["x"], y - o["y"]) - o["radius"]
            else:
                dx = max(abs(x - o["x"]) - o["half_x"], 0.0)
                dy = max(abs(y - o["y"]) - o["half_y"], 0.0)
                d = math.hypot(dx, dy)
            best = min(best, d)
        out.append(best)
    return np.array(out)


def _shade_modes(ax, x, modes):
    """Background bands coloured by controller mode, so reaction timing is
    readable against the clearance/yaw-rate curves."""
    i = 0
    n = len(modes)
    while i < n:
        j = i
        while j + 1 < n and modes[j + 1] == modes[i]:
            j += 1
        ax.axvspan(x[i], x[min(j + 1, n - 1)], color=MODE_COLOR.get(modes[i], "0.5"),
                   alpha=0.12, lw=0)
        i = j + 1


def plot_timeseries(runs, out):
    """Reaction-timing view v2 (Codex review). Per run, THREE stacked panels
    vs. SIM TIME t_capture (not corridor x, which mis-times stops/reversals),
    step-hold drawstyle so held commands read correctly:
      (1) SIGNED commanded yaw_rate + measured yaw_rate (finite-diff) -- shows
          left/right reversals the old abs()-vs-x view hid, and whether the
          vehicle actually tracks the command.
      (2) heading error (theta_target_body) that produces the command.
      (3) clearance-to-nearest-obstacle MINUS the airframe radius (true margin)
          and, when present, cross-track error from the reference route.
    Mode-shaded. Time on x so a stopped/reversing vehicle isn't misplaced."""
    n = len(runs)
    fig, axes = plt.subplots(n, 3, figsize=(15, 2.6 * n), squeeze=False)
    for row, run in zip(axes, runs):
        a_yaw, a_head, a_clr = row
        t = run["ts"]
        if len(t) < 2 or not np.isfinite(t).any():
            a_yaw.set_title(f"{run['name']} (too few timed frames)"); continue
        for ax in row:
            _shade_modes(ax, t, run["modes"]); ax.grid(alpha=0.2, lw=0.4)
        # (1) signed commanded vs measured yaw rate
        a_yaw.plot(t, run["yaw_rates"], color="#9467bd", lw=1.5, drawstyle="steps-post",
                   label="commanded yaw_rate")
        a_yaw.plot(t, run["meas_yaw"], color="#1f77b4", lw=1.2, alpha=0.8, label="measured yaw_rate")
        a_yaw.axhline(0.0, color="0.6", lw=0.6)
        a_yaw.set_ylabel("yaw rate (rad/s)", fontsize=8)
        st = _status(run["result"])
        a_yaw.set_title(f"{run['name']}  [{run['meta'].get('controller_version','?')}]  ({st})", fontsize=8)
        a_yaw.legend(fontsize=6, loc="upper left")
        # (2) heading error
        a_head.plot(t, run["heading_err"], color="#e377c2", lw=1.4, drawstyle="steps-post")
        a_head.axhline(0.0, color="0.6", lw=0.6)
        a_head.set_ylabel("heading error (rad)", fontsize=8)
        a_head.set_title("target heading error (drives yaw_rate)", fontsize=8)
        # (3) true clearance margin + cross-track
        clr = clearance_series(run["xs"], run["ys"]) - COLLISION_RADIUS_M
        a_clr.plot(t, clr, color="#2ca02c", lw=1.6, label="clearance - airframe radius (m)")
        a_clr.axhline(0.0, color="#d62728", ls="--", lw=0.9, label="contact (0 m)")
        if np.isfinite(run["cross_track"]).any():
            a_clr.plot(t, run["cross_track"], color="#ff7f0e", lw=1.2, alpha=0.85,
                       label="cross-track error (m)")
        a_clr.set_ylabel("m", fontsize=8); a_clr.set_title("clearance margin & cross-track", fontsize=8)
        a_clr.legend(fontsize=6, loc="upper right")
    for ax in axes[-1]:
        ax.set_xlabel("t_capture (s)")
    handles = [plt.Line2D([], [], color=c, lw=6, alpha=0.3, label=f"{m} mode") for m, c in MODE_COLOR.items()]
    fig.legend(handles=handles, loc="upper right", fontsize=7)
    fig.suptitle("Reaction timing v2: signed commanded vs measured yaw rate, heading error, "
                 "clearance margin & cross-track (vs sim time)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=125, bbox_inches="tight")
    print(f"saved {out}  ({n} run(s))")
    for run in runs:
        clr = clearance_series(run["xs"], run["ys"]) - COLLISION_RADIUS_M
        yr = np.abs(run["yaw_rates"])
        reversals = int(np.sum(np.diff(np.sign(run["yaw_rates"][np.abs(run["yaw_rates"]) > 0.05])) != 0))
        reacted = np.where(yr > 0.05)[0]
        first = f"{clr[reacted[0]]:.2f}m margin" if len(reacted) else "never reacted"
        print(f"  {run['name']:<22} status={_status(run['result']):<16} "
              f"min_margin={clr.min():+.2f}m  first_reaction@{first}  steering_reversals={reversals}")


def draw_obstacles(ax, xlim, ylim):
    for o in OBSTACLES:
        if not (xlim[0] - 2 <= o["x"] <= xlim[1] + 2):
            continue
        if o["kind"] == "circle":
            ax.add_patch(Circle((o["x"], o["y"]), o["radius"], color="0.35", zorder=2))
            ax.add_patch(Circle((o["x"], o["y"]), o["radius"] + COLLISION_RADIUS_M,
                                fill=False, ec="0.6", ls=":", lw=0.8, zorder=2))
        else:
            ax.add_patch(Rectangle((o["x"] - o["half_x"], o["y"] - o["half_y"]),
                                   2 * o["half_x"], 2 * o["half_y"], color="0.35", zorder=2))
            ax.add_patch(Rectangle((o["x"] - o["half_x"] - COLLISION_RADIUS_M,
                                    o["y"] - o["half_y"] - COLLISION_RADIUS_M),
                                   2 * (o["half_x"] + COLLISION_RADIUS_M),
                                   2 * (o["half_y"] + COLLISION_RADIUS_M),
                                   fill=False, ec="0.6", ls=":", lw=0.8, zorder=2))
        ax.annotate(o["name"], (o["x"], o["y"]), fontsize=5, color="w",
                    ha="center", va="center", zorder=3)


def draw_reference(ax, runs):
    """Draw the reference route(s) (A->B from meta) + goal-x line, so the
    intended path and where cross-track is measured from are explicit."""
    drawn_ref = set()
    for run in runs:
        gr = (run.get("meta") or {}).get("goal_reference") or {}
        if gr.get("kind") == "reference_path" and gr.get("A") and gr.get("B"):
            key = (tuple(gr["A"]), tuple(gr["B"]))
            if key not in drawn_ref:
                (ax_, ay_), (bx_, by_) = gr["A"], gr["B"]
                ax.plot([ax_, bx_], [ay_, by_], color="#17becf", ls="--", lw=1.2,
                        zorder=3, label="reference route" if not drawn_ref else None)
                drawn_ref.add(key)
        gx = (run.get("meta") or {}).get("goal_x_m")
        if gx is not None:
            ax.axvline(gx, color="0.5", ls=":", lw=0.9, zorder=1)


def plot_path(ax, run, single_color=None, label=None):
    xs, ys, modes = run["xs"], run["ys"], run["modes"]
    if len(xs) < 2:
        return
    if single_color is not None:  # overlay mode: one colour per run
        ax.plot(xs, ys, color=single_color, lw=1.4, zorder=4, label=label, alpha=0.9)
    else:             # per-run mode: colour segments by controller mode
        for i in range(len(xs) - 1):
            ax.plot(xs[i:i + 2], ys[i:i + 2],
                    color=MODE_COLOR.get(modes[i], "0.5"), lw=1.8, zorder=4)
    ax.scatter([xs[0]], [ys[0]], c="k", marker="o", s=30, zorder=5)          # start
    ax.scatter([xs[-1]], [ys[-1]], c=[single_color] if single_color is not None else "k", marker="s", s=30, zorder=5)  # end
    res = run["result"]
    if res.get("collided") and res.get("collision_xy"):
        cx, cy = res["collision_xy"]
        ax.scatter([cx], [cy], c="red", marker="X", s=90, zorder=6, edgecolors="k")


def bounds(runs):
    xs = np.concatenate([r["xs"] for r in runs if len(r["xs"])])
    ys = np.concatenate([r["ys"] for r in runs if len(r["ys"])])
    return (xs.min() - 3, xs.max() + 3), (ys.min() - 3, ys.max() + 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run dirs or *_frames.jsonl (globs ok)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--overlay", action="store_true", help="all paths on one map (default if >1 run and not --grid)")
    ap.add_argument("--grid", action="store_true", help="small-multiples, one map per run, mode-coloured")
    ap.add_argument("--timeseries", action="store_true",
                    help="reaction-timing view: clearance & |yaw_rate| vs x per run (not a map)")
    args = ap.parse_args()

    paths = []
    for p in args.runs:
        paths.extend(sorted(glob.glob(p)) if any(c in p for c in "*?[") else [p])
    runs = [r for r in (load_run(p) for p in paths) if r]
    if not runs:
        raise SystemExit("no runs with usable frames.jsonl + vehicle_position found")

    if args.timeseries:
        plot_timeseries(runs, args.out)
        return

    xlim, ylim = bounds(runs)
    use_grid = args.grid or (not args.overlay and len(runs) > 6)

    if use_grid:
        n = len(runs)
        cols = min(3, n); rows = math.ceil(n / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 3.4 * rows), squeeze=False)
        for ax, run in zip(axes.flat, runs):
            draw_obstacles(ax, xlim, ylim)
            draw_reference(ax, [run])
            plot_path(ax, run)
            clr = min_clearance(run["xs"], run["ys"]) - COLLISION_RADIUS_M
            ax.set_title(f"{run['name']}\nmin margin {clr:.2f} m | {_status(run['result'])}", fontsize=8)
            ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_aspect("equal"); ax.grid(alpha=0.2, lw=0.4)
        for ax in axes.flat[len(runs):]:
            ax.axis("off")
        # mode legend
        handles = [plt.Line2D([], [], color=c, lw=3, label=m) for m, c in MODE_COLOR.items()]
        fig.legend(handles=handles, loc="upper right", fontsize=8, title="controller mode")
        fig.suptitle("Trajectories (top-down), coloured by controller mode", fontsize=11)
    else:
        fig, ax = plt.subplots(figsize=(11, 6))
        draw_obstacles(ax, xlim, ylim)
        draw_reference(ax, runs)
        cmap = plt.cm.viridis(np.linspace(0, 1, len(runs)))
        for run, color in zip(runs, cmap):
            clr = min_clearance(run["xs"], run["ys"]) - COLLISION_RADIUS_M
            tag = "  X" if _status(run["result"]) == "COLLIDED" else ""
            plot_path(ax, run, single_color=color,
                      label=f"{run['name']} [{run['meta'].get('controller_version','?')}] "
                            f"(min margin {clr:.2f}m, {_status(run['result'])}){tag}")
        ax.set_xlim(xlim); ax.set_ylim(ylim); ax.set_aspect("equal"); ax.grid(alpha=0.2, lw=0.4)
        ax.set_xlabel("x (m, along corridor)"); ax.set_ylabel("y (m, lateral)")
        ax.set_title("Drone trajectories over obstacle map (o=start, s=end, X=collision)")
        ax.legend(fontsize=7, loc="best")

    fig.tight_layout()
    fig.savefig(args.out, dpi=130, bbox_inches="tight")
    print(f"saved {args.out}  ({len(runs)} run(s): {', '.join(r['name'] for r in runs)})")
    for r in runs:
        clr = min_clearance(r["xs"], r["ys"]) - COLLISION_RADIUS_M
        print(f"  {r['name']:<24} min_margin={clr:+5.2f}m  frames={len(r['xs']):<5} "
              f"{_status(r['result'])}  [{r['meta'].get('controller_version','?')}]")


if __name__ == "__main__":
    main()
