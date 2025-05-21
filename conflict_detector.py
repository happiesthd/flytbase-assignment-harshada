# conflict_detector.py

import numpy as np
from scipy.interpolate import interp1d
from typing import List, Tuple, Dict
from models import Drone, Waypoint


class ConflictDetector:
    """
    Detects and resolves conflicts between a primary drone and other drones.
    Uses linear interpolation of waypoints and a safety buffer in 3D space.
    """
    def __init__(self, safety_buffer: float = 1.0, time_resolution: float = 0.1):
        self.safety_buffer = safety_buffer
        self.time_resolution = time_resolution

    def interpolate_path(self, drone: Drone) -> Tuple[np.ndarray, np.ndarray]:
        """
        Linearly interpolates a drone's waypoints over its mission window.
        Returns:
          t_array: 1D array of sampled times
          path:    Nx3 array of [x, y, z] positions
        """
        if len(drone.waypoints) < 2 or drone.end_time is None:
            return np.array([]), np.array([[]])

        # extract coordinates
        xs = [wp.x for wp in drone.waypoints]
        ys = [wp.y for wp in drone.waypoints]
        zs = [wp.z for wp in drone.waypoints]

        # compute segment distances
        dists = [
            np.linalg.norm((xs[i]-xs[i-1], ys[i]-ys[i-1], zs[i]-zs[i-1]))
            for i in range(1, len(xs))
        ]
        total_dist = sum(dists)
        duration = drone.end_time - drone.start_time
        speed = total_dist / duration if duration > 0 else 0.0

        # assign times to each waypoint
        times_wp = [drone.start_time]
        cum = drone.start_time
        for d in dists:
            cum += (d / speed if speed > 0 else 0.0)
            times_wp.append(cum)

        # normalize to end exactly at end_time
        factor = duration / (times_wp[-1] - times_wp[0]) if times_wp[-1] != times_wp[0] else 1.0
        times_wp = [
            drone.start_time + (t - drone.start_time) * factor
            for t in times_wp
        ]

        # sample times uniformly
        num = int(duration / self.time_resolution) + 1
        t_array = np.linspace(drone.start_time, drone.end_time, num)

        # create interpolation functions
        fx = interp1d(times_wp, xs, fill_value="extrapolate", bounds_error=False)
        fy = interp1d(times_wp, ys, fill_value="extrapolate", bounds_error=False)
        fz = interp1d(times_wp, zs, fill_value="extrapolate", bounds_error=False)

        # build path
        path = np.vstack((fx(t_array), fy(t_array), fz(t_array))).T
        return t_array, path

    def check_conflict(
        self, primary: Drone, others: List[Drone]
    ) -> Tuple[str, List[Dict]]:
        """
        Checks for any time intervals when the primary drone
        encroaches within safety_buffer of any other drone.
        Returns:
          status:   "clear" or "conflict"
          conflicts: list of dicts with conflict segments
        """
        pt, pp = self.interpolate_path(primary)
        if pp.size == 0:
            return "error", []

        conflicts: List[Dict] = []
        for od in others:
            ot, op = self.interpolate_path(od)
            if op.size == 0:
                continue

            # find overlap in time
            t0 = max(primary.start_time, od.start_time)
            t1 = min(primary.end_time, od.end_time)
            if t0 >= t1:
                continue

            # mask to overlapping segment
            mask_p = (pt >= t0) & (pt <= t1)
            mask_o = (ot >= t0) & (ot <= t1)
            tp = pt[mask_p]; pp_seg = pp[mask_p]
            to = ot[mask_o]; op_seg = op[mask_o]

            # resample other drone onto primary times
            try:
                fx = interp1d(to, op_seg[:,0], fill_value="extrapolate", bounds_error=False)
                fy = interp1d(to, op_seg[:,1], fill_value="extrapolate", bounds_error=False)
                fz = interp1d(to, op_seg[:,2], fill_value="extrapolate", bounds_error=False)
                op_on_p = np.vstack((fx(tp), fy(tp), fz(tp))).T

                # distance computation
                dists = np.linalg.norm(pp_seg - op_on_p, axis=1)
                idx = np.where(dists < self.safety_buffer)[0]
            except Exception:
                continue

            if idx.size:
                # group consecutive indices into segments
                segments = []
                curr = [idx[0]]
                for i in idx[1:]:
                    if i == curr[-1] + 1:
                        curr.append(i)
                    else:
                        segments.append(curr)
                        curr = [i]
                segments.append(curr)

                # record each segment
                for seg in segments:
                    i0, i1 = seg[0], seg[-1]
                    conflicts.append({
                        'drone_id': od.mission_id,
                        'time_start': float(tp[i0]),
                        'time_end':   float(tp[i1]),
                        'pos_start': pp_seg[i0].tolist(),
                        'pos_end':   pp_seg[i1].tolist(),
                        'min_dist':  float(dists[seg].min())
                    })

        status = "clear" if not conflicts else "conflict"
        return status, conflicts

    def resolve_by_delay(
        self, primary: Drone, conflicts: List[Dict], others: List[Drone]
    ) -> float:
        """
        Finds the minimal delay (in seconds) to apply to primary
        so that no conflicts remain. Returns the delay.
        """
        delay = 0.0
        max_delay = 60.0  # safety cap

        while delay < max_delay:
            # clone primary with added delay
            shifted = Drone(
                primary.mission_id,
                waypoints=[Waypoint(wp.x, wp.y, wp.z) for wp in primary.waypoints],
                start_time=primary.start_time + delay,
                end_time=primary.end_time + delay
            )
            status, _ = self.check_conflict(shifted, others)
            if status == "clear":
                return delay
            delay += self.time_resolution

        return delay
