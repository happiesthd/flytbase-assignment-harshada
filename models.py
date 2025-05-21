# models.py

from typing import List, Optional, Tuple


class Waypoint:
    """
    Represents a 3D point in space, optionally tagged with a timestamp.
    """
    def __init__(self, x: float, y: float, z: float, time: Optional[float] = None):
        self.x = x
        self.y = y
        self.z = z
        self.time = time

    def __repr__(self) -> str:
        return f"Waypoint(x={self.x}, y={self.y}, z={self.z}, time={self.time})"


class Drone:
    """
    Encapsulates a drone mission, with waypoints and a mission time window.
    """
    def __init__(
        self,
        mission_id: str,
        waypoints: Optional[List[Waypoint]] = None,
        start_time: float = 0.0,
        end_time: Optional[float] = None
    ):
        self.mission_id = mission_id
        self.waypoints: List[Waypoint] = waypoints if waypoints else []
        self.start_time = start_time
        self.end_time = end_time
        self.color: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # assigned in simulator

    def add_waypoint(self, x: float, y: float, z: float, time: Optional[float] = None) -> None:
        """
        Append a waypoint to this drone's mission plan.
        """
        self.waypoints.append(Waypoint(x, y, z, time))

    def set_mission_window(self, start: float, end: float) -> None:
        """
        Define the start and end times of the mission.
        """
        self.start_time = start
        self.end_time = end

    def __repr__(self) -> str:
        return f"Drone({self.mission_id}, WPs={len(self.waypoints)}, {self.start_time}-{self.end_time})"
