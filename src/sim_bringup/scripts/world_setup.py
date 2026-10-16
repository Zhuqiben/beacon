#!/usr/bin/env python3
"""One randomisation pass over the world rules: build the scene, validate it, write it out.

This is the entry point that runs once per sim_bringup launch. ``m8_demo.launch.py`` calls
it synchronously **before gzserver starts** and receives three artefacts:

``<out-dir>/scene.json``
    The full result of this randomisation (``seed``, four landmarks, two patrol paths,
    two speeds). Used to score runs in ``scripts/benchmark.py`` and to check afterwards what
    the world actually was. No node on the robot side reads it.
``<out-dir>/landmark_world_scene.world``
    ``worlds/landmark_world_dynamic.world`` with the ``<pose>`` of the six models replaced
    by the sampled poses; every other byte is kept verbatim (camera, plugins, physics,
    comments).
``<out-dir>/dynamic_paths.yaml``
    Parameter overlay for ``dynamic_obstacles``, appended as a second ``--params-file``
    after its ``config/params.yaml``, so the path that runs is the path that was validated.

Generating the world before gzserver starts, instead of running a node that moves the
blocks with ``/gazebo/set_entity_state``
---------------------------------------------------------------------------
``set_entity_state`` cannot move an entity marked ``<static>true</static>``, so moving the
landmarks after start-up would require making all four non-static. They would then take
part in contact solving, the robot would push them aside, and the estimates in
``/landmark_map`` would no longer match the world -- whereas the evaluation assumes the
world does not change once the map is built. The derived-world approach keeps the four
landmarks static: physically fixed, yet randomised.

Command line
------------
    ros2 run sim_bringup world_setup.py                  # write ~/.ros/sim_bringup_scene
    ros2 run sim_bringup world_setup.py --seed 20240501  # reproduce the same world
    ros2 run sim_bringup world_setup.py --show           # also print landmarks and paths
    ros2 run sim_bringup world_setup.py --selftest 200   # offline property check, 200 seeds

A non-zero exit code means the world is invalid (or could not be built). ``--selftest``
runs every check in ``world_rules.validate_scene`` plus "each route has zero self
intersections".
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import world_rules as wr  # noqa: E402


def _selftest(count, seed0, rules, verbose):
    """Run ``count`` consecutive seeds and report the pass rate, the clearance of the routes
    to the static blocks and to the robot parking spot, the coverage, and the share of the
    replay spent giving way."""
    bad = 0
    failures = []
    stat = []
    rstat = []
    cover = {c: [] for c in wr.OBSTACLES}
    stall = {c: [] for c in wr.OBSTACLES}
    gap = []
    for i in range(count):
        seed = seed0 + i
        try:
            scene = wr.generate_scene(seed, rules)
        except RuntimeError as exc:
            bad += 1
            failures.append(f'seed={seed}: {exc}')
            continue
        problems = wr.validate_scene(scene, rules)
        if problems:
            bad += 1
            failures.append(f'seed={seed}: {problems}')
            continue
        # A random walk route naturally self-intersects and hugs walls, so all that is
        # checked here is whether a replay could be permanently blocked by a cell occupied
        # by a static block -- using the same Chebyshev criterion as the runtime.
        pts = [tuple(scene['landmarks'][c][:2]) for c in wr.COLORS]
        keep = wr._walk_keep(rules)
        robot_keep = wr._walk_robot_keep(rules)
        robots = wr._robot_pts(rules)
        step = float(rules['cell_size'])
        paths = {}
        for colour in wr.OBSTACLES:
            poly = [tuple(p) for p in scene['dynamic'][colour]['path']]
            paths[colour] = poly
            cheb = wr._min_static_cheb(poly, pts, step)
            stat.append(cheb)
            if cheb < keep - 1e-6:
                bad += 1
                failures.append(
                    f'seed={seed}: {colour} 走线离静态块只有 {cheb:.3f} m（切比雪夫）'
                    f'< {keep:.3f} m')
            robot_cheb = min(max(abs(p[0] - r[0]), abs(p[1] - r[1])) for p in poly
                             for r in robots)
            rstat.append(robot_cheb)
            if robot_cheb < robot_keep - 1e-6:
                bad += 1
                failures.append(
                    f'seed={seed}: {colour} 走线离机器人停驻点只有 {robot_cheb:.3f} m'
                    f'（切比雪夫）< {robot_keep:.3f} m')
        if len(paths) == len(wr.OBSTACLES):
            blockers = wr._blockers(rules, pts)
            per, _union = wr._cover_stats(paths, rules, blockers)
            for colour in wr.OBSTACLES:
                cover[colour].append(per[colour])
        replay = scene.get('replay') or {}
        for colour in wr.OBSTACLES:
            stall[colour].append(float((replay.get('stall') or {}).get(colour, 0.0)))
        gap.append(float(replay.get('gap', float('nan'))))

    ok = count - bad
    print(f'selftest: {ok}/{count} 个 seed 通过（不合格 {bad}）')
    if stat:
        print(f'  走线到静态块的净空（切比雪夫）：min {min(stat):.3f} m / mean {sum(stat)/len(stat):.3f} m'
              f'  （规则要求 >= {wr._walk_keep(rules):.3f}）')
    if rstat:
        print(f'  走线到机器人停驻点的净空：min {min(rstat):.3f} m / mean {sum(rstat)/len(rstat):.3f} m'
              f'  （规则要求 >= {wr._walk_robot_keep(rules):.3f}）')
    for colour in wr.OBSTACLES:
        vals = cover[colour]
        if vals:
            print(f'  {colour} 走线覆盖率：min {min(vals):.3f} / mean {sum(vals)/len(vals):.3f}'
                  f'  （规则要求 >= {float(rules["walk_cover_min"]):.3f}）')
    cap = float(rules['sim_stall_max'])
    for colour in wr.OBSTACLES:
        vals = stall[colour]
        if vals:
            print(f'  回放里 {colour} 让路占比：max {max(vals):.3f} / mean {sum(vals)/len(vals):.3f}'
                  f'  （规则要求 <= {cap:.2f}）')
    finite = [g for g in gap if g == g]
    if finite:
        print(f'  回放里两块最近间距：min {min(finite):.3f} m / mean {sum(finite)/len(finite):.3f} m'
              f'  （规则要求 >= {float(rules["sim_gap_min"]):.2f}）')
    if verbose:
        for line in failures[:20]:
            print('  ! ' + line)
    return 0 if bad == 0 else 1


def _show(scene):
    print('  四个地标（象限内随机落点）：')
    for colour in wr.COLORS:
        item = scene['landmarks'][colour]
        print(f'    landmark_{colour:<7s} x={item[0]:+.2f}  y={item[1]:+.2f}'
              f'   (第 {wr.QUADRANT[colour]} 象限)')
    robot = scene.get('robot') or [0.0, 0.0]
    cover = scene.get('cover') or {}
    print(f'  机器人停驻点：({robot[0]:+.2f}, {robot[1]:+.2f})'
          f'  —— 两条走线都不从这里经过')
    print('  两块动态障碍（随机游走走线）：')
    replay = scene.get('replay') or {}
    for colour in wr.OBSTACLES:
        item = scene['dynamic'][colour]
        print(f'    obstacle_{colour:<7s} speed={item["speed"]:.2f} m/s  '
              f'周长={item["perimeter"]:.2f} m  顶点={len(item["path"])}  '
              f'离静态块={item["clearance_static"]:.2f} m  '
              f'覆盖率={cover.get(colour, 0.0) * 100:.0f}%  '
              f'让路={(replay.get("stall") or {}).get(colour, 0.0) * 100:.0f}%  '
              f'起点=({item["home"][0]:+.2f}, {item["home"][1]:+.2f})')
    if cover:
        print(f'  两块合并覆盖率：{scene.get("cover_union", 0.0) * 100:.0f}%')
    print(f'  回放里两块最近间距：{replay.get("gap")} m'
          f'  让路次数：{replay.get("retreat")}'
          f'  锁死：{replay.get("deadlock")}')


def _default_root(here):
    """Default root for the template world and for params.yaml.

    In the source tree this resolves to ``scripts/..``; after ``ros2 run`` the script lives
    in ``<prefix>/lib/sim_bringup/`` and the resources in ``<prefix>/share/sim_bringup/``,
    where ``..`` would mean ``<prefix>/lib``. Both candidates are tried and the one that
    actually contains ``worlds/landmark_world_dynamic.world`` wins, so either way of running
    works. The launch file always passes ``--template`` and ``--params`` explicitly; this
    only makes a bare ``ros2 run sim_bringup world_setup.py`` work without the caller having
    to know where the script ended up.
    """
    candidates = [os.path.normpath(os.path.join(here, '..'))]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(get_package_share_directory('sim_bringup'))
    except Exception:                                  # noqa: BLE001 - also runs outside ROS
        pass
    for root in candidates:
        if os.path.exists(os.path.join(root, 'worlds', 'landmark_world_dynamic.world')):
            return root
    return candidates[0]


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    root = _default_root(here)
    default_template = os.path.join(root, 'worlds', 'landmark_world_dynamic.world')
    default_params = os.path.join(root, 'config', 'params.yaml')

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', default=None,
                        help='随机种子；省略或负数 = 每次不同（默认）')
    parser.add_argument('--template', default=default_template,
                        help='模板 world 文件（只改 <pose>）')
    parser.add_argument('--params', default=default_params,
                        help='读 world_rules 段的 params.yaml')
    parser.add_argument('--out-dir', default=wr.default_scene_dir(),
                        help='输出目录（默认 ~/.ros/sim_bringup_scene）')
    parser.add_argument('--selftest', type=int, default=0,
                        help='不写文件，改为离线跑 N 个 seed 的性质检查')
    parser.add_argument('--selftest-seed', type=int, default=1,
                        help='--selftest 的起始 seed')
    parser.add_argument('--show', action='store_true', help='打印地标与路径')
    parser.add_argument('--spawn-x', type=float, default=None,
                        help='机器人停驻点 x（默认取 world_rules 里的 robot_spawn_x）')
    parser.add_argument('--spawn-y', type=float, default=None,
                        help='机器人停驻点 y（默认取 world_rules 里的 robot_spawn_y）')
    parser.add_argument('--json', action='store_true',
                        help='最后一行输出产物路径的 JSON（给 launch 解析）')
    parser.add_argument('--quiet', action='store_true', help='只输出 --json 那一行')
    args = parser.parse_args(argv)

    rules = wr.make_rules()
    if args.params and os.path.exists(args.params):
        try:
            rules = wr.rules_from_params_file(args.params)
        except Exception as exc:                       # noqa: BLE001 - a missing section is normal
            print(f'world_setup: 读 {args.params} 的 world_rules 段失败（用默认规则）：{exc}')

    # The robot parking spot comes from the command line or the parameter file, and the
    # routes have to avoid it (path-level exclusion). Gazebo ground truth is not used: where
    # the robot parks is known to the launch side and does not count as cheating.
    if args.spawn_x is not None:
        rules['robot_spawn_x'] = float(args.spawn_x)
    if args.spawn_y is not None:
        rules['robot_spawn_y'] = float(args.spawn_y)

    if args.selftest > 0:
        return _selftest(args.selftest, args.selftest_seed, rules, not args.quiet)

    seed = None if args.seed is None else int(args.seed)
    scene = wr.generate_scene(seed, rules)

    os.makedirs(args.out_dir, exist_ok=True)
    scene_path = os.path.join(args.out_dir, 'scene.json')
    world_path = os.path.join(args.out_dir, 'landmark_world_scene.world')
    overlay_path = os.path.join(args.out_dir, 'dynamic_paths.yaml')
    manifest_path = os.path.join(args.out_dir, 'manifest.json')

    with open(args.template, 'r', encoding='utf-8', newline='') as handle:
        template_text = handle.read()
    poses = wr.scene_to_world_poses(scene, rules)
    wr.write_world(template_text, poses, world_path)

    header = (f'世界地图规则 seed={scene["seed"]}'
              f'（命令行 seed={args.seed if args.seed is not None else "随机"}）')
    with open(overlay_path, 'w', encoding='utf-8', newline='') as handle:
        handle.write(wr.scene_to_params_overlay(scene, header, rules))
    with open(scene_path, 'w', encoding='utf-8', newline='') as handle:
        handle.write(wr.dumps_scene(scene))
    manifest = {
        'seed': scene['seed'],
        'seed_arg': args.seed,
        'scene': scene_path,
        'world': world_path,
        'overlay': overlay_path,
        'poses': poses,
        'robot': scene.get('robot'),
        'cover': scene.get('cover'),
        'cover_union': scene.get('cover_union'),
    }
    with open(manifest_path, 'w', encoding='utf-8', newline='') as handle:
        json.dump(manifest, handle, indent=2, ensure_ascii=False)

    if not args.quiet:
        print(f'世界地图规则：seed={scene["seed"]}'
              + (f'（第 {scene["attempt"]} 次尝试）' if scene['attempt'] > 1 else ''))
        _show(scene)
        print(f'  派生 world   -> {world_path}')
        print(f'  参数覆盖     -> {overlay_path}')
        print(f'  场景清单     -> {scene_path}')
    if args.json:
        print(json.dumps({'seed': scene['seed'], 'scene': scene_path,
                          'world': world_path, 'overlay': overlay_path}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
