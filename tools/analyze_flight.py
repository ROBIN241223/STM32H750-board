#!/usr/bin/env python3
"""Offline analysis of a flight log written by gazebo_sim/controller/px4_x500.py.

Usage:
  tools/analyze_flight.py logs/flight-20260928-180000.csv [--threshold 10] [--plot]

Answers the questions the 2 Hz stdout line cannot:
  * where along the mission did the airframe tilt, and which leg
  * how far did the C estimator yaw drift from the Gazebo pose yaw
  * did the position/altitude loops hold their setpoints
  * ASCII top-down path so a LOITER circle can be eyeballed without a plot lib
"""

from __future__ import annotations

import argparse
import math
import os
import sys

LEG_NAMES = {-1: 'DONE', 16: 'WAYPOINT', 18: 'LOITER', 19: 'LOITERTIME',
             20: 'RTL', 21: 'LAND', 22: 'TAKEOFF'}
GUARD_NAMES = {0: 'NORM', 1: 'STAB', 2: 'RECOV'}


def load(path):
    meta, fields, rows = {}, [], []
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            line = line.rstrip('\n')
            if not line:
                continue
            if line.startswith('#'):
                body = line.lstrip('#').strip()
                if body.startswith('summary '):
                    for tok in body[len('summary '):].split():
                        k, _, v = tok.partition('=')
                        meta['summary_' + k] = v
                    continue
                if body.startswith('columns '):
                    fields = body[len('columns '):].split()
                    continue
                parts = body.split(None, 1)
                meta[parts[0]] = parts[1] if len(parts) > 1 else ''
                continue
            if not fields:
                fields = line.split(',')
                continue
            vals = line.split(',')
            if len(vals) != len(fields):
                continue
            try:
                rows.append({f: float(v) for f, v in zip(fields, vals)})
            except ValueError:
                continue
    return meta, fields, rows


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def section(title):
    print('\n== %s ' % title + '=' * max(0, 66 - len(title)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+', help='flight log(s) to analyze')
    ap.add_argument('--threshold', type=float, default=8.0,
                    help='tilt (deg) considered noteworthy [default 8]')
    ap.add_argument('--plot', action='store_true', help='ASCII top-down xy path')
    ap.add_argument('--top', type=int, default=25, help='max tilt rows to print')
    args = ap.parse_args()

    rc = 0
    for path in args.csv:
        if not os.path.exists(path):
            print('missing %s' % path, file=sys.stderr)
            rc = 1
            continue
        meta, fields, rows = load(path)
        if not rows:
            print('%s: no data rows' % path, file=sys.stderr)
            rc = 1
            continue

        print('#' * 72)
        print('# %s  (%d samples, %.1f s)' % (path, len(rows), rows[-1]['t'] - rows[0]['t']))
        print('#' * 72)
        if meta:
            section('run metadata')
            for k, v in meta.items():
                print('  %-18s %s' % (k, v))

        section('mission legs')
        print('%-10s %6s %7s %8s %9s %9s %9s %8s' %
              ('leg', 't0[s]', 'dur[s]', 'minz', 'maxtilt', 'maxroll', 'maxpitch', 'maxyawE'))
        legs = []
        for r in rows:
            key = int(r['leg'])
            if not legs or legs[-1][0] != key:
                legs.append([key, r['t'], r['t'], 1.0e9, 0.0, 0.0, 0.0, 0.0])
            s = legs[-1]
            s[2] = r['t']
            s[3] = min(s[3], r['z'])
            s[4] = max(s[4], math.degrees(math.hypot(math.radians(r['roll_deg']),
                                                     math.radians(r['pitch_deg']))))
            s[5] = max(s[5], abs(r['roll_deg']))
            s[6] = max(s[6], abs(r['pitch_deg']))
            s[7] = max(s[7], abs(wrap180(r['yaw_pose_deg'] - r['yaw_est_deg'])))
        for key, t0, t1, minz, mtilt, mroll, mpitch, myaw in legs:
            print('%-10s %6.1f %7.1f %8.2f %9.1f %9.1f %9.1f %8.1f' %
                  (LEG_NAMES.get(key, str(key)), t0, t1 - t0, minz, mtilt, mroll, mpitch, myaw))

        section('tilt above threshold (%.1f deg)' % args.threshold)
        hot = [r for r in rows
               if math.degrees(math.hypot(math.radians(r['roll_deg']),
                                          math.radians(r['pitch_deg']))) > args.threshold]
        if not hot:
            print('  (none)')
        else:
            print('  %d/%d samples (%.1f%%)' % (len(hot), len(rows),
                                                 100.0 * len(hot) / len(rows)))
            worst = sorted(hot, key=lambda r: -math.hypot(r['roll_deg'], r['pitch_deg']))
            print('  %-8s %-10s %7s %7s %8s %7s %7s %7s %8s' %
                  ('t[s]', 'leg', 'x', 'y', 'z', 'roll', 'pitch', 'yawE', 'gz'))
            for r in worst[:args.top]:
                print('  %-8.1f %-10s %7.2f %7.2f %8.2f %7.1f %7.1f %7.1f %8.3f' %
                      (r['t'], LEG_NAMES.get(int(r['leg']), r['leg']), r['x'], r['y'], r['z'],
                       r['roll_deg'], r['pitch_deg'],
                       wrap180(r['yaw_pose_deg'] - r['yaw_est_deg']), r['gyr_z']))

        section('yaw estimator error (est - pose)')
        errs = [wrap180(r['yaw_pose_deg'] - r['yaw_est_deg']) for r in rows]
        if errs:
            mean = sum(errs) / len(errs)
            var = sum((e - mean) ** 2 for e in errs) / len(errs)
            print('  mean=%+.1f deg  std=%.1f deg  min=%+.1f  max=%+.1f' %
                  (mean, math.sqrt(var), min(errs), max(errs)))
            # drift: mean signed error over 5 s buckets, so slow yaw wander shows up
            bucket = 5.0
            nb = int(max(1, rows[-1]['t'] / bucket))
            print('  %-10s %s' % ('window', ' '.join('%+6.1f' % _bucket_mean(errs, rows, b, bucket)
                                                     for b in range(nb))))
        else:
            print('  (no samples)')

        section('estimator attitude vs truth (roll/pitch)')
        er = [r['roll_deg'] - r['roll_est_deg'] for r in rows]
        ep = [r['pitch_deg'] - r['pitch_est_deg'] for r in rows]
        if er:
            print('  roll  err mean=%+.2f max|%.2f| deg' %
                  (sum(er) / len(er), max(abs(e) for e in er)))
            print('  pitch err mean=%+.2f max|%.2f| deg' %
                  (sum(ep) / len(ep), max(abs(e) for e in ep)))

        section('setpoint tracking (position loop error)')
        ex = [math.hypot(r['x'] - r['sp_x'], r['y'] - r['sp_y']) for r in rows]
        ez = [r['z'] - r['sp_z'] for r in rows]
        print('  xy err mean=%.2f max=%.2f m   z err mean=%+.2f max|%+.2f| m' %
              (sum(ex) / len(ex), max(ex), sum(ez) / len(ez), max(abs(v) for v in ez)))
        wbar = [(r['m0'] + r['m1'] + r['m2'] + r['m3']) / 4.0 for r in rows]
        thr = [r['thr'] for r in rows]
        print('  motor w mean=%.0f min=%.0f max=%.0f rad/s   collective mean=%.3f' %
              (sum(wbar) / len(wbar), min(wbar), max(wbar), sum(thr) / len(thr)))

        section('guard / land detector activity')
        for name, codes in (('guard', GUARD_NAMES), ('ld', {0: 'none', 1: 'GC', 2: 'ML', 3: 'LD'})):
            seen, cur, t_enter = [], None, 0.0
            for r in rows:
                v = int(r[name])
                if v != cur:
                    if cur is not None:
                        seen.append((t_enter, codes.get(cur, cur), r['t'] - t_enter))
                    cur, t_enter = v, r['t']
            if cur is not None:
                seen.append((t_enter, codes.get(cur, cur), rows[-1]['t'] - t_enter))
            interesting = [s for s in seen if s[1] not in ('NORM', 'none')]
            if interesting:
                for t0, c, dur in interesting:
                    print('  %-6s %-5s from t=%6.1fs  for %.1fs' % (name, c, t0, dur))
            else:
                print('  %-6s clean (no trips)' % name)

        if args.plot:
            section('xy path (top-down, +x right / +y up)')
            print(_ascii_path(rows))

        print('')
    return rc


def _bucket_mean(errs, rows, b, width):
    lo, hi = b * width, (b + 1) * width
    sel = [e for e, r in zip(errs, rows) if lo <= r['t'] < hi]
    return sum(sel) / len(sel) if sel else float('nan')


def _ascii_path(rows, width=68, height=21):
    xs = [r['x'] for r in rows]
    ys = [r['y'] for r in rows]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    span = max(x1 - x0, y1 - y0, 1.0)
    grid = [[' '] * width for _ in range(height)]
    for r in rows:
        cx = int((r['x'] - x0) / span * (width - 1))
        cy = int((y1 - r['y']) / span * (height - 1))
        grid[cy][cx] = 'o' if r['z'] < 0.5 else ('#' if r['z'] > 2.0 else '*')
    out = ['x:[%.1f..%.1f] y:[%.1f..%.1f]' % (x0, x1, y0, y1)]
    out += ['' .join(row) for row in grid]
    return '\n'.join(out)


if __name__ == '__main__':
    sys.exit(main())
