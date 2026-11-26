"""Unit tests for servo_logic (run with pytest, or `python3 test_servo_logic.py`)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from arm_hardware.servo_logic import (TrajectoryPlayer, format_command,  # noqa: E402
                                      rad_to_servo_deg)


def test_zero_rad_is_servo_centre():
    assert rad_to_servo_deg(0.0) == 90.0


def test_plus_minus_90_deg_maps_to_ends():
    assert abs(rad_to_servo_deg(1.5707963) - 180.0) < 1e-3
    assert abs(rad_to_servo_deg(-1.5707963) - 0.0) < 1e-3


def test_clamp():
    assert rad_to_servo_deg(3.0) == 180.0
    assert rad_to_servo_deg(-3.0) == 0.0


def test_direction_flip():
    assert abs(rad_to_servo_deg(0.5, direction=-1.0) -
               (90.0 - 28.6479)) < 1e-3


def test_format_command():
    assert format_command([90.0, 45.4, 179.6]) == 'A,90,45,180\n'


def test_player_interpolates_and_finishes():
    p = TrajectoryPlayer(['joint1', 'joint2'])
    assert p.load(['joint1'], [(2.0, [1.0])], now=0.0) == 1
    assert abs(p.step(1.0)['joint1'] - 0.5) < 1e-9
    assert p.current['joint2'] == 0.0           # untouched joint holds
    assert p.step(2.5)['joint1'] == 1.0
    assert not p.active


def test_player_continues_from_current_position():
    p = TrajectoryPlayer(['joint1'])
    p.load(['joint1'], [(2.0, [1.0])], now=0.0)
    p.step(1.0)                                  # at 0.5
    p.load(['joint1'], [(1.0, [0.0])], now=1.0)  # new goal from 0.5
    assert abs(p.step(1.5)['joint1'] - 0.25) < 1e-9


def test_player_zero_time_is_immediate():
    p = TrajectoryPlayer(['joint1'])
    p.load(['joint1'], [(0.0, [0.7])], now=0.0)
    assert p.step(0.0)['joint1'] == 0.7


def test_unknown_joints_ignored():
    p = TrajectoryPlayer(['joint1'])
    assert p.load(['nope'], [(1.0, [1.0])], now=0.0) == 0


def test_multi_point():
    p = TrajectoryPlayer(['joint1'])
    p.load(['joint1'], [(1.0, [1.0]), (2.0, [0.0])], now=0.0)
    assert abs(p.step(0.5)['joint1'] - 0.5) < 1e-9
    assert abs(p.step(1.5)['joint1'] - 0.5) < 1e-9


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
