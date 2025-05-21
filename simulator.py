# simulator.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.colors as mcolors

from models import Drone
from conflict_detector import ConflictDetector


class AirspaceSimulator:
    """
    Animates drone trajectories in 3D, highlights conflicts,
    and generates both pre- and post-resolution animations,
    preserving original conflict markers in both.
    """
    def __init__(self, drones: list[Drone], safety_buffer: float = 1.0, dt: float = 0.1):
        self.drones = drones
        self.detector = ConflictDetector(safety_buffer, dt)
        self.dt = dt

        # assign each drone a distinct color
        palette = list(mcolors.TABLEAU_COLORS.values())
        for i, d in enumerate(self.drones):
            d.color = palette[i % len(palette)]

    def _prepare_paths(self, primary_id: str, delay: float):
        primary = next(d for d in self.drones if d.mission_id == primary_id)
        shifted = Drone(
            primary.mission_id,
            start_time=primary.start_time + delay,
            end_time=primary.end_time + delay
        )
        for wp in primary.waypoints:
            shifted.add_waypoint(wp.x, wp.y, wp.z)

        others = [d for d in self.drones if d.mission_id != primary_id]
        all_drones = [shifted] + others

        paths = {}
        for d in all_drones:
            t, pos = self.detector.interpolate_path(d)
            if t.size and pos.size:
                paths[d.mission_id] = (t, pos, d.color, d is shifted)

        t0 = min(d.start_time for d in all_drones)
        t1 = max(d.end_time for d in all_drones)
        times = np.linspace(t0, t1, int((t1 - t0) / self.dt) + 1)

        return paths, times, shifted, others

    def _initialize_figure(self, paths: dict):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        all_coords = np.vstack([pos for (_, pos, *_ ) in paths.values()])
        buf = self.detector.safety_buffer
        mins = all_coords.min(axis=0) - buf
        maxs = all_coords.max(axis=0) + buf

        ax.set_xlim(mins[0], maxs[0])
        ax.set_ylim(mins[1], maxs[1])
        ax.set_zlim(mins[2], maxs[2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Drone Paths with Conflict Detection')

        time_txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
        ax.text2D(0.02, 0.91, "Blue = primary path", transform=ax.transAxes, color='blue')

        markers = {}
        for mid, (tarr, pos, col, isprim) in paths.items():
            style = '-' if isprim else '--'
            width = 2 if isprim else 1
            color = 'blue' if isprim else col
            ax.plot(pos[:, 0], pos[:, 1], pos[:, 2],
                    style, color=color, alpha=0.7, linewidth=width)
            markers[mid], = ax.plot([], [], [], 'o', color=color,
                                    markersize=(8 if isprim else 5))

        return fig, ax, time_txt, markers

    def _create_animation(self, fig, ax, time_txt, markers,
                          paths, times, conflict_segs):
        conflict_pts = []

        def update(frame: int):
            # clear old conflict points
            for pt in conflict_pts:
                pt.remove()
            conflict_pts.clear()

            t_now = times[frame]
            time_txt.set_text(f"t = {t_now:.1f}s")

            # move each drone marker
            for mid, (tarr, pos, _, _) in paths.items():
                if t_now < tarr[0] or t_now > tarr[-1]:
                    markers[mid].set_data([], [])
                    markers[mid].set_3d_properties([])
                else:
                    idx = np.abs(tarr - t_now).argmin()
                    x, y, z = pos[idx]
                    markers[mid].set_data([x], [y])
                    markers[mid].set_3d_properties([z])

            # overlay conflict segments only when within their interval
            for seg in conflict_segs:
                t0, t1 = seg['t0'], seg['t1']
                if t0 <= t_now <= t1:
                    frac = ((t_now - t0) / (t1 - t0)) if (t1 > t0) else 0
                    x = seg['p0'][0] + frac * (seg['p1'][0] - seg['p0'][0])
                    y = seg['p0'][1] + frac * (seg['p1'][1] - seg['p0'][1])
                    z = seg['p0'][2] + frac * (seg['p1'][2] - seg['p0'][2])
                    pt = ax.scatter([x], [y], [z],
                                    color='red', s=800, alpha=0.3)
                    conflict_pts.append(pt)

            return list(markers.values()) + [time_txt] + conflict_pts

        return FuncAnimation(fig, update,
                             frames=len(times),
                             interval=100,
                             blit=False)

    def animate(self, primary_id: str, delay: float,
                save_as: str, conflict_segs: list[dict]):
        paths, times, _, _ = self._prepare_paths(primary_id, delay)
        if not paths:
            print("No paths to animate.")
            return

        fig, ax, time_txt, markers = self._initialize_figure(paths)
        ani = self._create_animation(fig, ax, time_txt, markers,
                                     paths, times, conflict_segs)

        print(f"Saving animation to {save_as}…")
        ani.save(save_as, writer=PillowWriter(fps=10))
        plt.close(fig)
        print(f"Saved {save_as}")

    def run_both(
        self,
        primary_id: str,
        save_conflict: str = "mission_conflict.gif",
        save_resolved: str = "mission_resolved.gif"
    ) -> None:
        """
        Performs conflict check, prints details, resolves by delay,
        and creates two animations:
         1) with original timing (showing conflicts)
         2) with delay applied (showing same conflict markers)
        """
        primary = next(d for d in self.drones if d.mission_id == primary_id)
        others  = [d for d in self.drones if d.mission_id != primary_id]

        status, conflicts = self.detector.check_conflict(primary, others)
        print(f"Conflict status: {status}, segments: {len(conflicts)}")

        # print details if any
        if status == "conflict":
            print("Detailed conflict information:")
            for idx, seg in enumerate(conflicts, start=1):
                t0 = seg['time_start']
                t1 = seg['time_end']
                p0 = seg['pos_start']
                p1 = seg['pos_end']
                dist = seg['min_dist']
                print(f"  Segment {idx}:")
                print(f"    Time interval: [{t0:.2f}s — {t1:.2f}s]")
                print(f"    Start position: ({p0[0]:.1f}, {p0[1]:.1f}, {p0[2]:.1f})")
                print(f"    End   position: ({p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f})")
                print(f"    Minimum distance: {dist:.2f}\n")

        # prepare conflict segments dicts
        conflict_segs = [
            { 't0': c['time_start'], 't1': c['time_end'],
              'p0': c['pos_start'], 'p1': c['pos_end'] }
            for c in conflicts
        ]

        # 1) Conflict animation (no delay)
        self.animate(primary_id, delay=0.0,
                     save_as=save_conflict,
                     conflict_segs=conflict_segs)

        if status == "conflict":
            # resolve by delay
            delay = self.detector.resolve_by_delay(primary, conflicts, others)
            print(f"Applying delay of {delay:.1f}s to resolve conflicts\n")

            # 2) Resolved animation (same markers, shifted path)
            self.animate(primary_id, delay=delay,
                         save_as=save_resolved,
                         conflict_segs=conflict_segs)
        else:
            print("No conflicts detected. Only conflict animation generated.")
