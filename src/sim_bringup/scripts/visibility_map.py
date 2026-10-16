#!/usr/bin/env python3
"""Visibility / blind-spot analysis for the landmark arena.

For every candidate robot position inside the arena, sweep `--headings` orientations
and record the largest number of landmarks that fall inside the camera FOV **and**
inside the usable range window. Report the fraction of positions where >= 2 landmarks
(and >= 3 landmarks) can be seen simultaneously.

>= 2 is the hard requirement for EKF-SLAM: two range-bearing observations are needed to
solve x/y/theta. With only one, the pose degenerates to pure odometry and drifts.

Usage:
    python3 visibility_map.py --size 8 --fov 1.9199
    python3 visibility_map.py --size 10 --fov 1.50098
    python3 visibility_map.py --size 8 --fov 1.9199 --min-coverage 100
"""

import argparse
import math
import sys


def quadrant_centers(size):
    """Landmarks sit at the centre of each quadrant: (+-size/4, +-size/4)."""
    q = size / 4.0
    return [(-q, -q), (q, -q), (-q, q), (q, q)]


def analyze(size, fov, step, min_r, max_r, margin=0.35, headings=144):
    half = fov / 2.0
    landmarks = quadrant_centers(size)
    lo, hi = -size / 2.0 + margin, size / 2.0 - margin
    total = ok2 = ok3 = 0
    worst = 99
    blind = []

    y = lo
    while y <= hi + 1e-9:
        x = lo
        while x <= hi + 1e-9:
            total += 1
            best = 0
            for h in range(headings):
                th = -math.pi + 2.0 * math.pi * h / headings
                c = 0
                for lx, ly in landmarks:
                    d = math.hypot(lx - x, ly - y)
                    if d > max_r or d < min_r:
                        continue
                    rel = math.atan2(ly - y, lx - x) - th
                    rel = (rel + math.pi) % (2 * math.pi) - math.pi
                    if abs(rel) <= half:
                        c += 1
                if c > best:
                    best = c
            if best >= 2:
                ok2 += 1
            if best >= 3:
                ok3 += 1
            worst = min(worst, best)
            if best < 2:
                blind.append((round(x, 2), round(y, 2)))
            x += step
        y += step

    print(f'场地 {size}x{size} m，地标 {landmarks}')
    print(f'FOV {math.degrees(fov):.1f} 度（{fov:.5f} rad），测距 [{min_r}, {max_r}] m')
    pct2 = 100.0 * ok2 / total
    pct3 = 100.0 * ok3 / total
    print(f'  能看到 >=2 地标的位置占比: {pct2:.1f}%   (>=3: {pct3:.1f}%)')
    print(f'  最差位置的最大可见数: {worst}')
    if blind:
        print(f'  盲区格点数: {len(blind)}/{total}，示例: {blind[:5]}')
    else:
        print('  无盲区：任意可站立位置都能通过旋转看到 >=2 个地标')
    return pct2, pct3


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--size', type=float, default=8.0, help='arena side length [m]')
    p.add_argument('--fov', type=float, default=1.9199, help='horizontal FOV [rad]')
    p.add_argument('--step', type=float, default=0.25, help='grid pitch [m]')
    p.add_argument('--min-range', type=float, default=0.4, help='min usable range [m]')
    p.add_argument('--max-range', type=float, default=6.0, help='max usable range [m]')
    p.add_argument('--margin', type=float, default=0.35, help='wall clearance [m]')
    p.add_argument('--headings', type=int, default=144, help='orientation samples')
    p.add_argument('--min-coverage', type=float, default=0.0,
                   help='exit non-zero if the >=2 coverage is below this percentage (0 = report only)')
    a = p.parse_args()

    pct2, pct3 = analyze(a.size, a.fov, a.step, a.min_range, a.max_range,
                         margin=a.margin, headings=a.headings)

    # With --min-coverage this becomes a machine-checkable gate on the shipped geometry.
    if a.min_coverage > 0.0 and pct2 + 1e-9 < a.min_coverage:
        print(f'FAIL: coverage {pct2:.1f}% < required {a.min_coverage:.1f}%')
        return 1
    if a.min_coverage > 0.0:
        print(f'PASS: coverage {pct2:.1f}% >= required {a.min_coverage:.1f}%')
    return 0


if __name__ == '__main__':
    sys.exit(main())
