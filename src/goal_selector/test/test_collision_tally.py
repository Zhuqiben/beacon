#!/usr/bin/env python3
"""Offline unit tests for collision_tally (pure logic, no ROS or Gazebo needed)."""
from goal_selector.collision_tally import (
    BLOCK_MODELS, DEFAULT_REARM_SEC, STATIC_MODELS, CollisionTally, blocks_in_contact)


# ---------------- blocks_in_contact ----------------

def test_scoped_names_map_to_models():
    names = ['landmark_red::link::c', 'waffle::base_link::base_collision',
             'obstacle_white::link::box']
    assert blocks_in_contact(names) == {'landmark_red', 'obstacle_white'}


def test_robot_wall_and_empty_names_ignored():
    names = ['', None, 'waffle::base_link::base_collision', 'ground_plane::link::surface',
             'wall_north::link::c']
    assert blocks_in_contact(names) == set()


def test_all_six_blocks_recognised():
    assert blocks_in_contact(f'{m}::link::c' for m in BLOCK_MODELS) == set(BLOCK_MODELS)
    assert len(BLOCK_MODELS) == 6 and len(STATIC_MODELS) == 4


# ---------------- rising-edge counting ----------------

def test_rising_edge_counts_once():
    t = CollisionTally()
    t.reset('red', now=0.0)
    assert t.observe({'landmark_red'}, now=0.01) == 1
    assert t.observe({'landmark_red'}, now=0.02) == 0   # still in contact this frame
    assert t.observe({'landmark_red'}, now=0.30) == 0
    assert t.total == 1


def test_chatter_inside_rearm_window_is_one_collision():
    """ODE chatter: contact and separation alternate with a tiny gap, so only one hit."""
    t = CollisionTally(rearm_sec=0.35)
    t.reset('red', now=0.0)
    got = 0
    for i in range(20):
        got += t.observe({'landmark_red'}, now=0.001 * i)
        t.observe(set(), now=0.001 * i + 0.0005)
    assert got == 1 and t.total == 1


def test_real_second_hit_after_rearm_is_counted():
    t = CollisionTally(rearm_sec=0.35)
    t.reset('red', now=0.0)
    assert t.observe({'obstacle_black'}, now=0.0) == 1
    assert t.observe(set(), now=0.1) == 0
    assert t.observe({'obstacle_black'}, now=0.2) == 0    # too soon, merged with the previous hit
    t.observe(set(), now=0.3)
    assert t.observe({'obstacle_black'}, now=0.8) == 1    # past the window, a new collision
    assert t.total == 2


def test_two_blocks_in_one_frame_count_twice():
    t = CollisionTally()
    t.reset('red', now=0.0)
    assert t.observe({'landmark_red', 'obstacle_white'}, now=0.0) == 2
    assert t.summary()['static_collisions'] == 1
    assert t.summary()['dynamic_collisions'] == 1


def test_observation_without_active_trip_is_not_counted():
    t = CollisionTally()
    assert t.observe({'landmark_red'}, now=0.0) == 0
    assert t.total == 0


# ---------------- per-goal isolation and reset ----------------

def test_reset_zeroes_and_increments_trip():
    t = CollisionTally()
    t.reset('red', now=0.0)
    t.observe({'landmark_red'}, now=0.0)
    t.stop(now=1.0)
    assert t.summary()['collisions'] == 1
    t.reset('blue', now=2.0)
    assert t.total == 0
    assert t.summary() == {
        'trip': 2, 'target_color': 'blue', 'collisions': 0,
        'static_collisions': 0, 'dynamic_collisions': 0,
        'by_block': {}, 'elapsed_sec': 0.0}


def test_residual_contact_at_new_trip_is_not_a_collision():
    """Still pressing a block when the previous goal stopped: not a hit for the new goal."""
    t = CollisionTally()
    t.reset('red', now=0.0)
    t.observe({'landmark_red'}, now=0.0)         # first collision of that goal
    t.stop(now=1.0)
    t.reset('green', now=1.1)                    # landmark_red still pressed
    assert t.observe({'landmark_red'}, now=1.1) == 0
    assert t.total == 0
    t.observe(set(), now=1.5)                    # moved away
    assert t.observe({'landmark_red'}, now=3.0) == 1   # hit again, counts for this goal
    assert t.total == 1


def test_stop_is_idempotent_for_reads_and_freezes_elapsed():
    t = CollisionTally()
    t.reset('yellow', now=10.0)
    t.observe({'landmark_yellow'}, now=12.0)
    s = t.stop(now=15.0)
    assert s['elapsed_sec'] == 5.0
    assert t.stop(now=99.0)['elapsed_sec'] == 5.0   # a second stop does not move the stop time
    assert t.summary()['collisions'] == 1


# ---------------- summary text ----------------

def test_live_summary_tracks_elapsed():
    """Before the ledger is closed, summary(now) must report how long it has been running,
    otherwise the live report shows an elapsed time of 0 forever."""
    t = CollisionTally()
    t.reset('red', now=100.0)
    t.observe({'landmark_red'}, now=104.0)
    assert t.summary(now=112.5)['elapsed_sec'] == 12.5
    assert t.summary()['elapsed_sec'] == 0.0          # no now, and not closed yet
    assert t.stop(now=120.0)['elapsed_sec'] == 20.0
    assert t.summary(now=999.0)['elapsed_sec'] == 20.0  # now is ignored once closed


def test_describe_without_hits():
    t = CollisionTally()
    t.reset('red', now=0.0)
    assert t.describe() == '未与色块碰撞'


def test_describe_lists_static_and_dynamic():
    t = CollisionTally()
    t.reset('red', now=0.0)
    t.observe({'landmark_red'}, now=0.0)
    t.observe(set(), now=0.1)
    t.observe({'obstacle_white'}, now=1.0)
    t.observe(set(), now=1.1)
    t.observe({'obstacle_white'}, now=2.0)
    text = t.describe()
    assert text == '与色块碰撞 3 次（静态 1，动态 2：红块×1、白块(动态)×2）'


def test_defaults_are_used_when_not_overridden():
    t = CollisionTally()
    assert t.rearm_sec == DEFAULT_REARM_SEC
    assert t.models == BLOCK_MODELS
    assert t.trip == 0 and t.total == 0 and t.active is False
