#!/usr/bin/env python3
"""Optimize Numa's walking gait parameters for servo velocity.

Runs the real IK code (numa/IK.py walk_code) on the PC, samples one full
gait cycle, and computes the joint velocity each servo must sustain.
scipy's differential_evolution then searches the gait parameter space:

    travRate     stride half-length in mm (stride = 2 * travRate)
    loopLength   gait cycle period in ms
    FH           max foot lift height in mm
    down_frac    ALL_FEET_DOWN_TIME_FRAC (slow foot lower/raise fraction)
    trans_extra  TRANSITION_FRAC minus down_frac (fast lift/lower fraction)

Two modes:
  default            maximize body speed subject to servo velocity limits
  --target-speed S   hold body speed at S mm/s (loopLength derived from
                     travRate) and minimize the worst-case joint velocity
                     relative to its limit

Body speed for this gait is 4 * travRate / loopLength (mm/ms): feet in
stance sweep 2*travRate per half loop, and one leg pair is always down.

Note: IK.py/poses.py quantize positions to integer servo counts; this
script shadows `int` with `float` in those modules so velocities reflect
the smooth underlying trajectory instead of quantization steps.

Usage:
    python3 optimize_gait.py
    python3 optimize_gait.py --target-speed 120 --plot gait_vel.png
"""
import argparse
import os
import sys

import numpy as np
from scipy.optimize import differential_evolution

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'numa'))
import IK
import poses

# De-quantize servo positions (see module docstring)
IK.int = float
poses.int = float

AX_DEG_PER_COUNT = 300.0 / 1023.0
HX_DEG_PER_COUNT = 360.0 / 1500.0
AX_DEG_S_PER_SPEED_UNIT = 0.111 * 6.0  # MOVING_SPEED register unit -> deg/s

# Current on-robot values (init.py speeds, IK.py gait constants, numa.py travRate)
DEFAULT_LEG_LIMIT = 300 * AX_DEG_S_PER_SPEED_UNIT   # 199.8 deg/s (SERVO_SPEED)
DEFAULT_COAX_LIMIT = 200 * AX_DEG_S_PER_SPEED_UNIT  # 133.2 deg/s (COAX_SPEED)
BASELINE = dict(travRate=25.0, loopLength=650.0, foot_h=25.0,
                down_frac=0.12, trans_extra=0.28)

BAD_COST = 1e6


def make_gaits():
    leg_geom, l1, l2, l3, l4 = poses.gen_numa2_legs()
    return IK.Gaits(leg_geom, l1, l2, l3, l4)


def joint_scales(gaits):
    """Degrees per servo count for the 12 active joints (coax x4, femur x4, tibia x4)."""
    legs = [gaits.leg1, gaits.leg2, gaits.leg3, gaits.leg4]
    scales = []
    for joint in range(3):
        for leg in legs:
            kind = leg.servo_types[joint]
            scales.append(HX_DEG_PER_COUNT if kind.startswith('hx') else AX_DEG_PER_COUNT)
    return np.array(scales)


def get_now(ms, loopLength, half):
    # Mirrors NumaMain.get_now (numa.py)
    now2 = ms % loopLength
    now3 = (ms + half) % loopLength
    now4 = loopLength - ms % loopLength
    now1 = loopLength - (ms + half) % loopLength
    return now1, now2, now3, now4


def simulate(travRate, loopLength, foot_h, down_frac, trans_extra, n_samples=120):
    """Sample one gait cycle. Returns (times_ms, positions[n,12]) or (None, None)
    if the IK fails (leg over-extended / triangle inequality violated)."""
    IK.FH = foot_h
    IK.ALL_FEET_DOWN_TIME_FRAC = down_frac
    IK.TRANSITION_FRAC = down_frac + trans_extra
    gaits = make_gaits()
    half = loopLength / 2.0
    times = np.linspace(0.0, loopLength, n_samples, endpoint=False)
    pos = np.empty((n_samples, 12))
    for i, ms in enumerate(times):
        n1, n2, n3, n4 = get_now(ms, loopLength, half)
        try:
            gaits.walk_code(loopLength, half, travRate, n1, n2, n3, n4, 0)
        except ValueError:
            return None, None
        g = gaits
        pos[i] = (g.s11pos, g.s21pos, g.s31pos, g.s41pos,
                  g.s12pos, g.s22pos, g.s32pos, g.s42pos,
                  g.s13pos, g.s23pos, g.s33pos, g.s43pos)
    return times, pos


def joint_velocities(pos, loopLength, scales):
    """deg/s for each sample and joint, using cyclic forward differences."""
    dt_ms = loopLength / len(pos)
    vel_counts = (np.roll(pos, -1, axis=0) - pos) / dt_ms
    return vel_counts * scales * 1000.0


def evaluate(params, scales, n_samples=120):
    """Returns (speed_mm_s, peak_coax, peak_leg, vel) or None on IK failure."""
    travRate, loopLength, foot_h, down_frac, trans_extra = params
    _, pos = simulate(travRate, loopLength, foot_h, down_frac, trans_extra, n_samples)
    if pos is None:
        return None
    vel = joint_velocities(pos, loopLength, scales)
    peak_coax = np.max(np.abs(vel[:, :4]))
    peak_leg = np.max(np.abs(vel[:, 4:]))
    speed = 4000.0 * travRate / loopLength
    return speed, peak_coax, peak_leg, vel


def cost_max_speed(x, scales, coax_limit, leg_limit, n_samples):
    result = evaluate(x, scales, n_samples)
    if result is None:
        return BAD_COST
    speed, peak_coax, peak_leg, _ = result
    exc_c = max(0.0, peak_coax / coax_limit - 1.0)
    exc_l = max(0.0, peak_leg / leg_limit - 1.0)
    return -speed + 1e5 * (exc_c ** 2 + exc_l ** 2) + 1e3 * (exc_c + exc_l)


def cost_target_speed(x, target, scales, coax_limit, leg_limit, n_samples):
    travRate, foot_h, down_frac, trans_extra = x
    loopLength = 4000.0 * travRate / target
    if not 300.0 <= loopLength <= 3500.0:
        return BAD_COST
    result = evaluate((travRate, loopLength, foot_h, down_frac, trans_extra),
                      scales, n_samples)
    if result is None:
        return BAD_COST
    _, peak_coax, peak_leg, _ = result
    return max(peak_coax / coax_limit, peak_leg / leg_limit)


def report(label, params, scales, coax_limit, leg_limit, n_samples):
    result = evaluate(params, scales, n_samples)
    travRate, loopLength, foot_h, down_frac, trans_extra = params
    print("\n%s" % label)
    print("  travRate=%.1f mm  loopLength=%.0f ms  FH=%.1f mm  "
          "ALL_FEET_DOWN_TIME_FRAC=%.3f  TRANSITION_FRAC=%.3f"
          % (travRate, loopLength, foot_h, down_frac, down_frac + trans_extra))
    if result is None:
        print("  IK FAILS for these parameters (leg over-extended)")
        return None
    speed, peak_coax, peak_leg, vel = result
    print("  body speed: %.1f mm/s   (stride %.0f mm @ %.2f Hz)"
          % (speed, 2 * travRate, 1000.0 / loopLength))
    for name, peak, limit in (("coax     ", peak_coax, coax_limit),
                              ("femur/tib", peak_leg, leg_limit)):
        units = peak / AX_DEG_S_PER_SPEED_UNIT
        flag = "  <-- EXCEEDS LIMIT" if peak > limit else ""
        print("  peak %s vel: %6.1f deg/s  (limit %.1f, AX MOVING_SPEED ~%d)%s"
              % (name, peak, limit, round(units), flag))
    return result


def plot_velocities(path, params, scales, n_samples):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    travRate, loopLength = params[0], params[1]
    times, pos = simulate(*params, n_samples=n_samples)
    vel = joint_velocities(pos, loopLength, scales)
    frac = times / loopLength
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    groups = (("Coax", 0), ("Femur", 4), ("Tibia", 8))
    for ax, (name, base) in zip(axes, groups):
        for leg in range(4):
            ax.plot(frac, vel[:, base + leg], label="leg %d" % (leg + 1))
        ax.set_ylabel("%s deg/s" % name)
        ax.grid(True, alpha=0.3)
    axes[0].legend(ncol=4, fontsize=8)
    axes[0].set_title("Joint velocities, travRate=%.1f loopLength=%.0fms"
                      % (travRate, loopLength))
    axes[-1].set_xlabel("fraction of gait cycle")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print("\nVelocity plot written to %s" % path)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--target-speed", type=float, default=None, metavar="MM_S",
                   help="hold this body speed (mm/s) and minimize peak joint "
                        "velocity instead of maximizing speed")
    p.add_argument("--leg-limit", type=float, default=DEFAULT_LEG_LIMIT,
                   help="femur/tibia velocity limit in deg/s (default %.1f = "
                        "AX MOVING_SPEED 300)" % DEFAULT_LEG_LIMIT)
    p.add_argument("--coax-limit", type=float, default=DEFAULT_COAX_LIMIT,
                   help="coax velocity limit in deg/s (default %.1f = "
                        "AX MOVING_SPEED 200)" % DEFAULT_COAX_LIMIT)
    p.add_argument("--min-foot-h", type=float, default=8.0, metavar="MM",
                   help="minimum allowed foot lift height in mm (default 8). "
                        "The optimizer always pushes FH to this floor, so set "
                        "it to the clearance the terrain actually requires")
    p.add_argument("--samples", type=int, default=120,
                   help="samples per gait cycle (default 120)")
    p.add_argument("--maxiter", type=int, default=80,
                   help="differential_evolution iterations (default 80)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--plot", metavar="PNG",
                   help="write joint velocity plot for the optimized gait")
    args = p.parse_args()

    scales = joint_scales(make_gaits())

    baseline = (BASELINE['travRate'], BASELINE['loopLength'], BASELINE['foot_h'],
                BASELINE['down_frac'], BASELINE['trans_extra'])
    report("Baseline (current code, fastest loopLength):", baseline,
           scales, args.coax_limit, args.leg_limit, args.samples)

    if args.target_speed is None:
        print("\nOptimizing: maximize body speed with peak velocities within limits...")
        bounds = [(10, 50),      # travRate mm
                  (300, 3000),   # loopLength ms
                  (args.min_foot_h, 40),  # FH mm
                  (0.02, 0.25),  # down_frac
                  (0.05, 0.45)]  # trans_extra
        res = differential_evolution(
            cost_max_speed, bounds,
            args=(scales, args.coax_limit, args.leg_limit, args.samples),
            seed=args.seed, maxiter=args.maxiter, popsize=16, tol=1e-7,
            polish=True, workers=-1, updating='deferred')
        best = tuple(res.x)
    else:
        print("\nOptimizing: minimize peak joint velocity at %.1f mm/s..."
              % args.target_speed)
        bounds = [(10, 50),      # travRate mm
                  (args.min_foot_h, 40),  # FH mm
                  (0.02, 0.25),  # down_frac
                  (0.05, 0.45)]  # trans_extra
        res = differential_evolution(
            cost_target_speed, bounds,
            args=(args.target_speed, scales, args.coax_limit, args.leg_limit,
                  args.samples),
            seed=args.seed, maxiter=args.maxiter, popsize=16, tol=1e-7,
            polish=True, workers=-1, updating='deferred')
        travRate = res.x[0]
        best = (travRate, 4000.0 * travRate / args.target_speed,
                res.x[1], res.x[2], res.x[3])

    report("Optimized:", best, scales, args.coax_limit, args.leg_limit,
           args.samples)
    print("\nTo apply: set trav_rate_default (numa.py) and FH, "
          "ALL_FEET_DOWN_TIME_FRAC, TRANSITION_FRAC (IK.py); add/replace the "
          "loopLength in loopLengthList (numa.py).")

    if args.plot:
        plot_velocities(args.plot, best, scales, args.samples)


if __name__ == "__main__":
    main()
