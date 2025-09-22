import math
import json
import tkinter as tk
from typing import List, Tuple, Dict, Optional, Set
from geometry import (
    Point, is_simple_polygon, ear_clipping_triangulation, 
    three_color_from_triangulation, is_clockwise, is_simple_polygon
)
from viewport import Viewport
from utils import calculate_edge_lengths

class PolygonEditor:
    def __init__(self, canvas: tk.Canvas):
        self.canvas = canvas
        self.points: List[Point] = []
        self.closed: bool = False
        self.drag_index: Optional[int] = None
        self.selected_index: Optional[int] = None
        self.viewport = Viewport()
        self.triangles: List[Tuple[int, int, int]] = []
        self.diagonals: List[Tuple[int, int]] = []
        self.colors: Dict[int, int] = {}
        self.guards: List[int] = []
        # Socket on boundary: defined by edge index k and t in [0,1] on segment (k -> k+1)
        self.socket_edge: Optional[int] = None
        self.socket_t: float = 0.5
        # Costs
        self.cost_wire_per_unit: float = 1.0
        self.cost_camera: float = 1.0
        self.last_cost: Optional[float] = None
        # View options
        self.show_opt_only: bool = False
        # Stats for UI
        self.stats: Dict[str, float] = {}
        self.class_stats: Dict[int, Dict[str, float]] = {}

    def _refresh_view(self):
        """Redraw the entire canvas with current state."""
        self.canvas.delete("all")
        w = int(self.canvas.winfo_width())
        h = int(self.canvas.winfo_height())
        pts = self.points
        self.viewport.fit_points(pts, padding=20.0)

        # Draw diagonals (always show, even in optimal-only mode)
        for i, j in self.diagonals:
            self._draw_segment(pts[i], pts[j], w, h, fill="#bdc3c7", width=1, dash=(3, 3))

        # Draw polygon edges
        if len(pts) >= 2:
            for i in range(len(pts) - (0 if self.closed else 1)):
                a = pts[i]
                b = pts[(i + 1) % len(pts)]
                self._draw_segment(a, b, w, h, fill="#333333", width=2)

        # Draw vertices with color or only guards
        r = 5
        guard_set = set(self.guards)
        if self.show_opt_only:
            # Show all vertices; optimal green, others gray
            for idx, p in enumerate(pts):
                sx, sy = self.viewport.world_to_screen(p, w, h)
                col = "#27ae60" if idx in guard_set else "#bdc3c7"
                outline = "#f39c12" if self.selected_index == idx else "white"
                self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r, fill=col, outline=outline, width=2 if self.selected_index == idx else 1)
        else:
            for idx, p in enumerate(pts):
                sx, sy = self.viewport.world_to_screen(p, w, h)
                base = {0: "#e74c3c", 1: "#2980b9", 2: "#8e44ad"}.get(self.colors.get(idx, -1), "#7f8c8d")
                col = "#27ae60" if idx in guard_set else base
                outline = "#f39c12" if self.selected_index == idx else "white"
                self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r, fill=col, outline=outline, width=2 if self.selected_index == idx else 1)

        # Draw wires from guards to socket when present
        if self.socket_edge is not None and self.guards:
            n = len(pts)
            k = self.socket_edge % n
            a = pts[k]
            b = pts[(k + 1) % n]
            t = max(0.0, min(1.0, self.socket_t))
            socket_pt = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            edges, prefix = calculate_edge_lengths(self.points)
            perim = prefix[-1] if prefix else 0.0
            _, s_socket = self._socket_position_and_s()
            for gi in self.guards:
                # choose shorter boundary direction
                s_i = prefix[gi]
                ccw = (s_socket - s_i) % perim
                cw = (s_i - s_socket) % perim
                go_ccw = ccw <= cw
                path: List[Point] = []
                cur = gi
                if go_ccw:
                    while True:
                        path.append(pts[cur])
                        if cur == k:
                            break
                        cur = (cur + 1) % n
                else:
                    while True:
                        path.append(pts[cur])
                        prev = (cur - 1) % n
                        if prev == k:
                            break
                        cur = prev
                path.append(socket_pt)
                if len(path) >= 2:
                    coords: List[int] = []
                    for P in path:
                        x, y = self.viewport.world_to_screen(P, w, h)
                        coords.extend([x, y])
                    self.canvas.create_line(*coords, fill="#f1c40f", width=3)

        # Draw socket if set
        if self.socket_edge is not None and pts:
            n = len(pts)
            k = self.socket_edge % n
            a = pts[k]
            b = pts[(k + 1) % n]
            t = max(0.0, min(1.0, self.socket_t))
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            sx, sy = self.viewport.world_to_screen((x, y), w, h)
            self.canvas.create_rectangle(sx - 5, sy - 5, sx + 5, sy + 5, outline="#8e44ad", fill="#dcd6f7", width=2)

        # Status text
        status = "" if not self.closed else ("Simple" if is_simple_polygon(pts) else "Non-simple")
        if self.last_cost is not None:
            status += f"  Cost={self.last_cost:.3f}"
        self.canvas.create_text(10, 10, anchor="nw", text=status, fill="#666666")

    def _draw_segment(self, a: Point, b: Point, w: int, h: int, **kwargs):
        """Helper to draw a line segment with viewport transformation."""
        x1, y1 = self.viewport.world_to_screen(a, w, h)
        x2, y2 = self.viewport.world_to_screen(b, w, h)
        self.canvas.create_line(x1, y1, x2, y2, **kwargs)

    def add_point(self, p: Point):
        if self.closed:
            return
        self.points.append(p)
        self._refresh_view()

    def close_polygon(self):
        if len(self.points) >= 3:
            self.closed = True
            self._refresh_view()

    def reset(self):
        self.points.clear()
        self.closed = False
        self.triangles.clear()
        self.diagonals.clear()
        self.colors.clear()
        self.guards.clear()
        self.socket_edge = None
        self.socket_t = 0.5
        self.last_cost = None
        self._refresh_view()

    def save_json(self, path: str):
        """Save polygon points to JSON file."""
        data = {
            "points": self.points,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def load_json(self, path: str):
        """Load polygon points from JSON file."""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        pts = data.get("points", [])
        self.points = [(float(x), float(y)) for x, y in pts]
        self.closed = len(self.points) >= 3
        self.triangles.clear()
        self.diagonals.clear()
        self.colors.clear()
        self.guards.clear()
        self._refresh_view()

    def _cost_for_set(self, indices: List[int]) -> Tuple[float, str, float, float, float]:
        camera_cost = self.cost_camera * len(indices)
        wire_cost = 0.0
        detail = ""
        _, s = self._socket_position_and_s()
        if s is None:
            total = camera_cost
            return total, f"No socket set; camera-only ({len(indices)} cams).", 0.0, camera_cost, 0.0

        # Sum minimal boundary distances
        dsum = 0.0
        edges, prefix = calculate_edge_lengths(self.points)
        perim = prefix[-1] if prefix else 0.0
        for i in indices:
            s_i = prefix[i]
            ccw = (s - s_i) % perim
            cw = (s_i - s) % perim
            dsum += min(ccw, cw)

        wire_cost = self.cost_wire_per_unit * dsum
        total = camera_cost + wire_cost
        detail = f"wire={wire_cost:.3f} (sum len {dsum:.3f}) + cams={camera_cost:.3f}"
        return total, detail, dsum, camera_cost, wire_cost

    def _socket_position_and_s(self) -> Tuple[Optional[Point], Optional[float]]:
        if self.socket_edge is None or len(self.points) < 2:
            return None, None
        n = len(self.points)
        k = self.socket_edge % n
        a = self.points[k]
        b = self.points[(k + 1) % n]
        t = max(0.0, min(1.0, self.socket_t))
        pos = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        edges, prefix = calculate_edge_lengths(self.points)
        s = prefix[k] + t * edges[k]  # arclength from vertex 0 along CCW
        return pos, s

    def compute(self) -> Tuple[bool, str]:
        if not self.closed:
            return False, "Close the polygon first."
        if not is_simple_polygon(self.points):
            return False, "Polygon is not simple (self-intersections)."
        
        # Triangulate
        tris, diags = ear_clipping_triangulation(self.points)
        if not tris or len(tris) < len(self.points) - 2:
            return False, "Triangulation failed. Try adjusting points."
            
        # Color vertices
        cols = three_color_from_triangulation(self.points, tris)
        if len(cols) != len(self.points):
            return False, "Coloring failed."
            
        # Choose optimal guard set
        classes: Dict[int, List[int]] = {0: [], 1: [], 2: []}
        for v, c in cols.items():
            classes[c].append(v)
        candidates = [classes[0], classes[1], classes[2]]
        
        # Default to minimal size if no costs
        best_choice = min(candidates, key=len)
        best_total = None
        best_detail = ""
        best_cls_id = None
        best_wire_len = 0.0
        best_wire_cost = 0.0
        best_cam_cost = 0.0
        
        # Evaluate costs per class
        self.class_stats = {}
        for cls_id, cls in enumerate([classes[0], classes[1], classes[2]]):
            total, detail, dsum, cam_c, wire_c = self._cost_for_set(cls)
            self.class_stats[cls_id] = {
                "size": float(len(cls)),
                "wire_len": float(dsum),
                "wire_cost": float(wire_c),
                "cam_cost": float(cam_c),
                "total_cost": float(total),
            }
            if best_total is None or total < best_total:
                best_total = total
                best_detail = f"chose color-class |V|={len(cls)}; {detail}"
                best_choice = cls
                best_cls_id = cls_id
                best_wire_len = dsum
                best_wire_cost = wire_c
                best_cam_cost = cam_c

        # Store results
        self.triangles = tris
        self.diagonals = diags
        self.colors = cols
        self.guards = sorted(best_choice)
        self.last_cost = best_total
        
        # Update stats
        self.stats = {
            "n": float(len(self.points)),
            "tri": float(max(len(self.points) - 2, 0)),
            "guards": float(len(best_choice)),
            "wire_len": float(best_wire_len),
            "wire_cost": float(best_wire_cost),
            "cam_cost": float(best_cam_cost),
            "total_cost": float(best_total if best_total is not None else 0.0),
            "class": float(best_cls_id if best_cls_id is not None else -1),
        }
        
        self._refresh_view()
        return True, ""
