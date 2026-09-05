#!/usr/bin/env python3
"""Visualize Numa's walking gait: baseline vs optimized parameters.

Steps the real IK (numa/IK.py walk_code) through a full gait cycle and
records one leg's linkage geometry (coax pivot, femur, knee, tibia, foot)
by instrumenting Gaits.genericLegKinem. Produces:

  - an animated GIF, both gaits side by side in real time (so cadence and
    stride differences are directly comparable), and
  - a static "strobe" PNG overlaying the linkage at phases through the
    cycle, colored by time.

Side view is the leg's vertical plane: x = distance from coax axis (mm),
y = height above ground (mm). The lower panel plots the foot's fore-aft
offset (trav) vs lift height - the classic foot-path diagram.

Usage:
    python3 viz_gait.py
    python3 viz_gait.py --opt 41.1 590 8.0 0.084 0.433 --gif fast.gif
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import optimize_gait as og

IK = og.IK

# Result of `optimize_gait.py --min-foot-h 25` (same foot clearance as baseline)
DEFAULT_OPT = (50.0, 1020.0, 25.0, 0.033, 0.449)


class RecordingGaits(IK.Gaits):
    """Gaits that records (footH, legLen, v23, v34) for each leg's IK call."""

    def genericLegKinem(self, footH, legLen, debug=0):
        result = IK.Gaits.genericLegKinem(self, footH, legLen, debug)
        self._rec.append((footH, legLen, tuple(self.v23), tuple(self.v34)))
        return result


def sample_leg(params, leg_index=2, n_samples=200):
    """Step one gait cycle, return (times_ms, chains[n,5,2], trav, footh).

    chains holds side-view xy of points 1 (coax), 2 (femur pivot), 3 (knee),
    4 (foot attach), 5 (foot tip); y is height above ground.
    """
    travRate, loopLength, foot_h, down_frac, trans_extra = params
    IK.FH = foot_h
    IK.ALL_FEET_DOWN_TIME_FRAC = down_frac
    IK.TRANSITION_FRAC = down_frac + trans_extra
    leg_geom, l1, l2, l3, l4 = og.poses.gen_numa2_legs()
    gaits = RecordingGaits(leg_geom, l1, l2, l3, l4)
    half = loopLength / 2.0
    times = np.linspace(0.0, loopLength, n_samples, endpoint=False)
    chains = np.empty((n_samples, 5, 2))
    trav = np.empty(n_samples)
    footh = np.empty(n_samples)
    for i, ms in enumerate(times):
        n1, n2, n3, n4 = og.get_now(ms, loopLength, half)
        gaits._rec = []
        gaits.walk_code(loopLength, half, travRate, n1, n2, n3, n4, 0)
        fh, legLen, v23, v34 = gaits._rec[leg_index]
        p1 = np.array([0.0, gaits.bodyH])
        p2 = p1 + [gaits.L12, 0.0]
        p3 = p2 + v23
        p4 = p3 + v34
        p5 = p4 - [0.0, gaits.L45]
        # Reconstruction must land the foot attach point where the IK put it
        assert abs(p4[0] - legLen) < 1e-3 and abs(p4[1] - (fh + gaits.L45)) < 1e-3, \
            (p4, legLen, fh)
        chains[i] = [p1, p2, p3, p4, p5]
        trav[i] = (gaits.trav1, gaits.trav2, gaits.trav3, gaits.trav4)[leg_index]
        footh[i] = fh
    return times, chains, trav, footh


def gait_label(name, params):
    travRate, loopLength = params[0], params[1]
    speed = 4000.0 * travRate / loopLength
    return "%s: %.0f mm/s\nstride %.0f mm, loop %.0f ms" % (
        name, speed, 2 * travRate, loopLength)


def setup_axes(fig, gaits_data):
    """2x2 grid: rows = side view / foot path, cols = baseline / optimized."""
    axes = fig.subplots(2, 2)
    xmax = max(d['chains'][:, :, 0].max() for d in gaits_data) + 15
    ymax = max(d['chains'][:, :, 1].max() for d in gaits_data) + 15
    tmax = max(np.abs(d['trav']).max() for d in gaits_data) + 5
    hmax = max(d['footh'].max() for d in gaits_data) + 5
    for col, d in enumerate(gaits_data):
        ax = axes[0][col]
        ax.set_xlim(-10, xmax)
        ax.set_ylim(-8, ymax)
        ax.set_aspect('equal')
        ax.axhline(0, color='saddlebrown', lw=2)
        ax.set_title(gait_label(d['name'], d['params']), fontsize=10)
        ax.set_ylabel("height mm" if col == 0 else "")
        ax = axes[1][col]
        ax.set_xlim(-tmax, tmax)
        ax.set_ylim(-3, hmax)
        ax.set_aspect('equal')
        ax.axhline(0, color='saddlebrown', lw=2)
        ax.set_xlabel("foot fore-aft offset mm")
        ax.set_ylabel("foot lift mm" if col == 0 else "")
    return axes


def make_strobe(path, gaits_data, n_poses=10):
    fig = plt.figure(figsize=(10, 7))
    axes = setup_axes(fig, gaits_data)
    cmap = plt.cm.viridis
    for col, d in enumerate(gaits_data):
        n = len(d['times'])
        for k in range(n_poses):
            i = k * n // n_poses
            c = cmap(k / n_poses)
            axes[0][col].plot(d['chains'][i, :, 0], d['chains'][i, :, 1],
                              '-o', color=c, ms=3, lw=1.5, alpha=0.8)
        axes[0][col].plot(d['chains'][:, 4, 0], d['chains'][:, 4, 1],
                          color='gray', lw=0.8, alpha=0.6)
        axes[1][col].plot(d['trav'], d['footh'], color='tab:blue', lw=1.2)
    fig.suptitle("Leg linkage through one gait cycle (color = phase, "
                 "dark early / bright late)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print("Strobe image written to %s" % path)


def make_gif(path, gaits_data, fps=30, n_cycles=2):
    fig = plt.figure(figsize=(10, 7))
    axes = setup_axes(fig, gaits_data)
    artists = []
    for col, d in enumerate(gaits_data):
        axes[0][col].plot(d['chains'][:, 4, 0], d['chains'][:, 4, 1],
                          color='gray', lw=0.8, alpha=0.6)
        chain, = axes[0][col].plot([], [], '-o', color='tab:red', ms=5, lw=2.5)
        axes[1][col].plot(d['trav'], d['footh'], color='lightgray', lw=1.2)
        dot, = axes[1][col].plot([], [], 'o', color='tab:red', ms=8)
        artists.append((chain, dot))
    total_ms = n_cycles * max(d['params'][1] for d in gaits_data)
    frame_ms = 1000.0 / fps
    n_frames = int(total_ms / frame_ms)
    fig.suptitle("Real-time comparison (both animations share the clock)")
    fig.tight_layout()

    def update(frame):
        t = frame * frame_ms
        out = []
        for d, (chain, dot) in zip(gaits_data, artists):
            n = len(d['times'])
            i = int((t % d['params'][1]) / d['params'][1] * n) % n
            chain.set_data(d['chains'][i, :, 0], d['chains'][i, :, 1])
            dot.set_data([d['trav'][i]], [d['footh'][i]])
            out.extend((chain, dot))
        return out

    anim = FuncAnimation(fig, update, frames=n_frames, blit=True)
    anim.save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print("Animation written to %s (%d frames, %.1f s real time)"
          % (path, n_frames, total_ms / 1000.0))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--opt", type=float, nargs=5, default=list(DEFAULT_OPT),
                   metavar=("TRAVRATE", "LOOP_MS", "FH", "DOWN_FRAC", "TRANS_EXTRA"),
                   help="optimized gait params (default: --min-foot-h 25 result)")
    p.add_argument("--leg", type=int, default=3, choices=(1, 2, 3, 4),
                   help="which leg to visualize (default 3, front-right)")
    p.add_argument("--gif", default="gait_compare.gif")
    p.add_argument("--png", default="gait_compare.png")
    p.add_argument("--fps", type=int, default=30)
    args = p.parse_args()

    baseline = (og.BASELINE['travRate'], og.BASELINE['loopLength'],
                og.BASELINE['foot_h'], og.BASELINE['down_frac'],
                og.BASELINE['trans_extra'])
    gaits_data = []
    for name, params in (("Baseline", baseline), ("Optimized", tuple(args.opt))):
        times, chains, trav, footh = sample_leg(params, leg_index=args.leg - 1)
        gaits_data.append(dict(name=name, params=params, times=times,
                               chains=chains, trav=trav, footh=footh))
    make_strobe(args.png, gaits_data)
    make_gif(args.gif, gaits_data, fps=args.fps)


if __name__ == "__main__":
    main()
