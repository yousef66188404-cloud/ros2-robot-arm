"""Pure-Python helpers for the hardware bridge (no ROS imports, easy to test).

* rad_to_servo_deg : radians -> servo degrees, with offset, direction and clamp
* format_command   : build the serial line sent to the firmware
* TrajectoryPlayer : linear interpolation of a JointTrajectory over time
"""
import math
from typing import Dict, List, Sequence, Tuple


def rad_to_servo_deg(rad: float, offset_deg: float = 90.0,
                     direction: float = 1.0,
                     min_deg: float = 0.0, max_deg: float = 180.0) -> float:
    """0 rad -> offset_deg (servo centre). Result is clamped to [min, max]."""
    deg = offset_deg + direction * math.degrees(rad)
    return max(min_deg, min(max_deg, deg))


def format_command(angles_deg: Sequence[float]) -> str:
    """Message format: ``A,<deg1>,<deg2>,...\\n`` with integer degrees."""
    return 'A,' + ','.join(str(int(round(a))) for a in angles_deg) + '\n'


class TrajectoryPlayer:
    """Plays trajectory points by interpolating linearly between them.

    The first segment starts from the arm's current position, so a new message
    always continues smoothly from where the previous one left off.
    """

    def __init__(self, joint_names: Sequence[str]):
        self.joint_names: List[str] = list(joint_names)
        self.current: Dict[str, float] = {j: 0.0 for j in self.joint_names}
        self._start: Dict[str, float] = dict(self.current)
        self._waypoints: List[Tuple[float, Dict[str, float]]] = []
        self._t0 = 0.0

    @property
    def active(self) -> bool:
        return bool(self._waypoints)

    def load(self, names: Sequence[str],
             points: Sequence[Tuple[float, Sequence[float]]], now: float) -> int:
        """Start playing ``points`` = [(time_from_start_s, positions), ...].

        Returns how many of the message's joints are known to this player
        (0 means the message was ignored).
        """
        index = {n: i for i, n in enumerate(names) if n in self.current}
        if not index or not points:
            return 0
        self._start = dict(self.current)
        self._t0 = now
        waypoints = []
        for t, pos in sorted(points, key=lambda p: p[0]):
            target = dict(self._start)  # joints not in the message hold position
            for joint, i in index.items():
                if i < len(pos):
                    target[joint] = float(pos[i])
            waypoints.append((float(t), target))
        self._waypoints = waypoints
        return len(index)

    def step(self, now: float) -> Dict[str, float]:
        """Advance to time ``now`` and return the current joint positions."""
        if not self._waypoints:
            return self.current
        elapsed = now - self._t0
        prev_t, prev = 0.0, self._start
        for i, (t, target) in enumerate(self._waypoints):
            if elapsed < t:
                alpha = (elapsed - prev_t) / (t - prev_t) if t > prev_t else 1.0
                self.current = {j: prev[j] + alpha * (target[j] - prev[j])
                                for j in self.joint_names}
                return self.current
            prev_t, prev = t, target
        self.current = dict(prev)
        self._waypoints = []
        return self.current
