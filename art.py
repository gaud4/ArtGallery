import json
import math
import sys
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Set


# ==============================
# Geometry primitives and utils
# ==============================

Point = Tuple[float, float]


def almost_equal(a: float, b: float, eps: float = 1e-9) -> bool:
    return abs(a - b) <= eps


def cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def orientation(a: Point, b: Point, c: Point) -> float:
    return cross(b[0] - a[0], b[1] - a[1], c[0] - a[0], c[1] - a[1])


def on_segment(a: Point, b: Point, p: Point) -> bool:
    if not almost_equal(orientation(a, b, p), 0.0):
        return False
    return min(a[0], b[0]) - 1e-9 <= p[0] <= max(a[0], b[0]) + 1e-9 and \
           min(a[1], b[1]) - 1e-9 <= p[1] <= max(a[1], b[1]) + 1e-9


def segments_intersect(a: Point, b: Point, c: Point, d: Point, *, proper: bool = True) -> bool:
    o1 = orientation(a, b, c)
    o2 = orientation(a, b, d)
    o3 = orientation(c, d, a)
    o4 = orientation(c, d, b)

    if (o1 > 0 and o2 < 0 or o1 < 0 and o2 > 0) and (o3 > 0 and o4 < 0 or o3 < 0 and o4 > 0):
        return True
    if not proper:
        return on_segment(a, b, c) or on_segment(a, b, d) or on_segment(c, d, a) or on_segment(c, d, b)
    return False


def is_simple_polygon(poly: List[Point]) -> bool:
    n = len(poly)
    if n < 3:
        return False
    # edges (i, i+1)
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            # skip adjacent edges and the edge sharing a vertex at wrap
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            b1, b2 = poly[j], poly[(j + 1) % n]
            if segments_intersect(a1, a2, b1, b2, proper=True):
                return False
    return True


def is_clockwise(poly: List[Point]) -> bool:
    # Signed area (standard shoelace). area2 > 0 => CCW, < 0 => CW
    area2 = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        area2 += x1 * y2 - x2 * y1
    return area2 < 0


def point_in_triangle(p: Point, a: Point, b: Point, c: Point) -> bool:
    # Barycentric technique with sign consistency
    o1 = orientation(a, b, p)
    o2 = orientation(b, c, p)
    o3 = orientation(c, a, p)
    has_neg = (o1 < -1e-12) or (o2 < -1e-12) or (o3 < -1e-12)
    has_pos = (o1 > 1e-12) or (o2 > 1e-12) or (o3 > 1e-12)
    return not (has_neg and has_pos)


def is_convex(prev: Point, curr: Point, nxt: Point, ccw: bool) -> bool:
    turn = orientation(prev, curr, nxt)
    return turn > 1e-12 if ccw else turn < -1e-12


# =========================
# Ear clipping triangulator
# =========================

def ear_clipping_triangulation(poly: List[Point]) -> Tuple[List[Tuple[int, int, int]], List[Tuple[int, int]]]:
    """Triangulate a simple polygon (no holes) by ear clipping.

    Returns (triangles, diagonals), where triangles are index triples into the original polygon
    (with original orientation), and diagonals are pairs of vertex indices used.
    """
    n = len(poly)
    if n < 3:
        return [], []

    # Ensure CCW orientation for algorithm stability
    verts = list(range(n))
    if is_clockwise(poly):
        verts.reverse()

    def diagonal_is_internal(i_idx: int, k_idx: int, working: List[int]) -> bool:
        a = poly[i_idx]
        b = poly[k_idx]
        m = len(working)
        # Check intersection with polygon edges except edges adjacent to the diagonal endpoints
        for t in range(m):
            u_idx = working[t]
            v_idx = working[(t + 1) % m]
            # Skip edges incident to a or b
            if u_idx in (i_idx, k_idx) or v_idx in (i_idx, k_idx):
                continue
            if segments_intersect(a, b, poly[u_idx], poly[v_idx], proper=True):
                return False
        return True

    def is_ear_at(pos: int, working: List[int]) -> bool:
        m = len(working)
        prev_idx = working[(pos - 1) % m]
        vi = working[pos]
        next_idx = working[(pos + 1) % m]
        a, b, c = poly[prev_idx], poly[vi], poly[next_idx]
        # convex corner in CCW polygon
        if not is_convex(a, b, c, ccw=True):
            return False
        # diagonal inside polygon
        if not diagonal_is_internal(prev_idx, next_idx, working):
            return False
        # No other vertex strictly inside or on edges of triangle
        for vk in working:
            if vk in (prev_idx, vi, next_idx):
                continue
            if point_in_triangle(poly[vk], a, b, c):
                return False
        return True

    triangles: List[Tuple[int, int, int]] = []
    diagonals: List[Tuple[int, int]] = []

    # Copy for safe iteration
    working = verts.copy()
    fail_safe = 0
    while len(working) > 3 and fail_safe < 10000:
        ear_found = False
        for i in range(len(working)):
            if is_ear_at(i, working):
                prev_i = working[(i - 1) % len(working)]
                vi = working[i]
                next_i = working[(i + 1) % len(working)]
                triangles.append((prev_i, vi, next_i))
                diagonals.append((prev_i, next_i))
                del working[i]
                ear_found = True
                break
        if not ear_found:
            # Fallback: try relaxed inside test (exclude near-collinear)
            for i in range(len(working)):
                prev_i = working[(i - 1) % len(working)]
                vi = working[i]
                next_i = working[(i + 1) % len(working)]
                a, b, c = poly[prev_i], poly[vi], poly[next_i]
                if not is_convex(a, b, c, ccw=True):
                    continue
                if not diagonal_is_internal(prev_i, next_i, working):
                    continue
                ok = True
                for vk in working:
                    if vk in (prev_i, vi, next_i):
                        continue
                    if point_in_triangle(poly[vk], a, b, c):
                        ok = False
                        break
                if ok:
                    triangles.append((prev_i, vi, next_i))
                    diagonals.append((prev_i, next_i))
                    del working[i]
                    ear_found = True
                    break
        if not ear_found:
            break
        fail_safe += 1

    if len(working) == 3:
        triangles.append((working[0], working[1], working[2]))

    # Remap triangles to original index order direction
    # If initial poly was CW, we reversed verts; triangles already refer to original indices though.
    return triangles, diagonals


# =========================
# 3-coloring via ear order
# =========================

def three_color_from_triangulation(poly: List[Point], triangles: List[Tuple[int, int, int]]) -> Dict[int, int]:
    """Color vertices 0,1,2 so every triangle has all three colors.

    Uses ear removal order: build an adjacency (ear stack) from triangles.
    """
    n = len(poly)
    if n < 3:
        return {}

    # Build ear-removal order by reversing triangulation fan: For each triangle, assume the middle vertex was ear tip.
    # To be robust, we reconstruct a removal sequence by finding a vertex shared by exactly one triangle iteratively.
    tri_list = triangles.copy()
    incident: Dict[int, Set[int]] = {i: set() for i in range(n)}
    for ti, (a, b, c) in enumerate(tri_list):
        incident[a].add(ti)
        incident[b].add(ti)
        incident[c].add(ti)

    removal_stack: List[Tuple[int, Tuple[int, int]]] = []  # (ear_tip, (nbr1, nbr2))
    remaining_vertices: Set[int] = set(range(n))

    # Copy of structures for mutation
    tri_alive: Set[int] = set(range(len(tri_list)))

    while len(remaining_vertices) > 3 and tri_alive:
        ear_found = False
        for v in list(remaining_vertices):
            # An ear tip appears in exactly one alive triangle if remaining is polygonal
            alive_tris = [ti for ti in incident[v] if ti in tri_alive]
            if len(alive_tris) != 1:
                continue
            t = alive_tris[0]
            a, b, c = tri_list[t]
            nbrs = [u for u in (a, b, c) if u != v]
            removal_stack.append((v, (nbrs[0], nbrs[1])))
            tri_alive.remove(t)
            remaining_vertices.remove(v)
            ear_found = True
            break
        if not ear_found:
            # Fallback: pick any vertex with minimal incident alive tris
            best_v = None
            best_count = 10**9
            for v in list(remaining_vertices):
                cnt = sum(1 for ti in incident[v] if ti in tri_alive)
                if cnt < best_count and cnt > 0:
                    best_count = cnt
                    best_v = v
            if best_v is None:
                break
            # Choose a triangle containing best_v
            t = next(ti for ti in incident[best_v] if ti in tri_alive)
            a, b, c = tri_list[t]
            nbrs = [u for u in (a, b, c) if u != best_v]
            removal_stack.append((best_v, (nbrs[0], nbrs[1])))
            tri_alive.remove(t)
            remaining_vertices.remove(best_v)

    # Color the base triangle
    colors: Dict[int, int] = {}
    base_vertices = list(remaining_vertices) if remaining_vertices else []
    if len(base_vertices) == 3:
        colors[base_vertices[0]] = 0
        colors[base_vertices[1]] = 1
        colors[base_vertices[2]] = 2

    # Add ears back in reverse removal order
    for v, (u1, u2) in reversed(removal_stack):
        cu1 = colors.get(u1)
        cu2 = colors.get(u2)
        if cu1 is None or cu2 is None:
            # pick colors for neighbors if missing (robustness)
            if cu1 is None:
                colors[u1] = len(colors) % 3
                cu1 = colors[u1]
            if cu2 is None:
                colors[u2] = (len(colors) + 1) % 3
                cu2 = colors[u2]
        # Assign the remaining color
        colors[v] = ({0, 1, 2} - {cu1, cu2}).pop()

    return colors


def choose_guards_from_coloring(colors: Dict[int, int]) -> List[int]:
    buckets: Dict[int, List[int]] = {0: [], 1: [], 2: []}
    for v, c in colors.items():
        buckets[c].append(v)
    # return smallest color class
    return min(buckets.values(), key=len)


# =========================
# GUI: Interactive Editor
# =========================


@dataclass
class Viewport:
    min_x: float = 0.0
    min_y: float = 0.0
    max_x: float = 1.0
    max_y: float = 1.0

    def fit_points(self, pts: List[Point], padding: float = 20.0):
        if not pts:
            self.min_x, self.min_y, self.max_x, self.max_y = 0, 0, 1, 1
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        if almost_equal(min_x, max_x):
            min_x -= 0.5
            max_x += 0.5
        if almost_equal(min_y, max_y):
            min_y -= 0.5
            max_y += 0.5
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.min_x -= padding
        self.min_y -= padding
        self.max_x += padding
        self.max_y += padding

    def world_to_screen(self, p: Point, width: int, height: int) -> Point:
        wx = (p[0] - self.min_x) / max(self.max_x - self.min_x, 1e-9)
        wy = (p[1] - self.min_y) / max(self.max_y - self.min_y, 1e-9)
        sx = int(wx * width)
        sy = int((1 - wy) * height)
        return (sx, sy)


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
        data = {
            "points": self.points,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def load_json(self, path: str):
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

    def compute(self) -> Tuple[bool, str]:
        if not self.closed:
            return False, "Close the polygon first."
        if not is_simple_polygon(self.points):
            return False, "Polygon is not simple (self-intersections)."
        tris, diags = ear_clipping_triangulation(self.points)
        if not tris or len(tris) < len(self.points) - 2:
            return False, "Triangulation failed. Try adjusting points."
        cols = three_color_from_triangulation(self.points, tris)
        if len(cols) != len(self.points):
            return False, "Coloring failed."
        # choose guards by cost if socket set, else by size
        chosen_guards, total_cost, detail = self._choose_guards_with_cost(cols)
        self.triangles = tris
        self.diagonals = diags
        self.colors = cols
        self.guards = sorted(chosen_guards)
        self.last_cost = total_cost
        self._refresh_view()
        base = ""
        # base = f"Computed {len(tris)} triangles; guards chosen: {len(self.guards)}."
        # if total_cost is not None:
        #     base += f" Total cost={total_cost:.3f}. {detail}"
        return True, base

    # ============ Cost evaluation ============
    def _boundary_lengths(self) -> Tuple[List[float], List[float]]:
        """Return edge lengths and prefix sums along boundary starting at vertex 0."""
        n = len(self.points)
        edges = []
        for i in range(n):
            a, b = self.points[i], self.points[(i + 1) % n]
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            edges.append(math.hypot(dx, dy))
        prefix = [0.0]
        for L in edges:
            prefix.append(prefix[-1] + L)
        return edges, prefix  # prefix length n+1 (last = perimeter)

    def _socket_position_and_s(self) -> Tuple[Optional[Point], Optional[float]]:
        if self.socket_edge is None or len(self.points) < 2:
            return None, None
        n = len(self.points)
        k = self.socket_edge % n
        a = self.points[k]
        b = self.points[(k + 1) % n]
        t = max(0.0, min(1.0, self.socket_t))
        pos = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        edges, prefix = self._boundary_lengths()
        s = prefix[k] + t * edges[k]  # arclength from vertex 0 along CCW
        return pos, s

    def _dist_along_boundary(self, i_from: int, s_to: float) -> float:
        """Shortest boundary path length from vertex i to socket arclength s_to (both directions on boundary).
        We assume wires run on boundary; choose min of CW/CCW distances.
        """
        edges, prefix = self._boundary_lengths()
        perim = prefix[-1]
        s_i = prefix[i_from]
        ccw = (s_to - s_i) % perim
        cw = (s_i - s_to) % perim
        return min(ccw, cw)

    def _cost_for_set(self, indices: List[int]) -> Tuple[float, str, float, float, float]:
        camera_cost = self.cost_camera * len(indices)
        wire_cost = 0.0
        detail = ""
        _, s = self._socket_position_and_s()
        if s is None:
            # No socket, only camera cost
            total = camera_cost
            return total, f"No socket set; camera-only ({len(indices)} cams).", 0.0, camera_cost, 0.0
        # sum minimal boundary distances
        dsum = 0.0
        for i in indices:
            d = self._dist_along_boundary(i, s)
            dsum += d
        wire_cost = self.cost_wire_per_unit * dsum
        total = camera_cost + wire_cost
        detail = f"wire={wire_cost:.3f} (sum len {dsum:.3f}) + cams={camera_cost:.3f}"
        return total, detail, dsum, camera_cost, wire_cost

    def _choose_guards_with_cost(self, colors: Dict[int, int]) -> Tuple[List[int], Optional[float], str]:
        # Default to minimal color class if no socket or no costs provided
        classes: Dict[int, List[int]] = {0: [], 1: [], 2: []}
        for v, c in colors.items():
            classes[c].append(v)
        candidates = [classes[0], classes[1], classes[2]]
        best_set: List[int] = min(candidates, key=len)
        # Evaluate costs per class and take min
        best_total = None
        best_detail = ""
        best_choice = best_set
        best_cls_id = None
        best_wire_len = 0.0
        best_wire_cost = 0.0
        best_cam_cost = 0.0
        # compute per-class stats regardless of socket presence
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
        # record stats
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
        return best_choice, best_total, best_detail

    # ============ Rendering ============
    def _refresh_view(self):
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
            edges, prefix = self._boundary_lengths()
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
        x1, y1 = self.viewport.world_to_screen(a, w, h)
        x2, y2 = self.viewport.world_to_screen(b, w, h)
        self.canvas.create_line(x1, y1, x2, y2, **kwargs)


class ArtGalleryApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Art Gallery Problem - Guards via Triangulation and 3-Coloring")
        self.root.minsize(900, 600)

        # --- Sidebar with scrollbar ---
        SIDEBAR_WIDTH = 270  # Fixed width for sidebar
        
        sidebar_frame = ttk.Frame(root, width=SIDEBAR_WIDTH)
        sidebar_frame.pack(side=tk.LEFT, fill=tk.Y)
        sidebar_frame.pack_propagate(False)  # Prevent frame from shrinking

        sidebar_canvas = tk.Canvas(sidebar_frame, borderwidth=0, background="#f8f8f8", width=SIDEBAR_WIDTH - 20)  # 20px for scrollbar
        sidebar_canvas.pack(side=tk.LEFT, fill=tk.Y, expand=True)

        scrollbar = ttk.Scrollbar(sidebar_frame, orient="vertical", command=sidebar_canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        sidebar_canvas.configure(yscrollcommand=scrollbar.set)

        self.sidebar = ttk.Frame(sidebar_canvas, width=SIDEBAR_WIDTH - 20)  # Match canvas width
        self.sidebar.bind(
            "<Configure>",
            lambda e: sidebar_canvas.configure(scrollregion=sidebar_canvas.bbox("all"))
        )
        
        sidebar_canvas.create_window((0, 0), window=self.sidebar, anchor="nw", width=SIDEBAR_WIDTH - 20)  # Fix window width

        # Bind mouse wheel events to the canvas
        def _on_mousewheel(event):
            sidebar_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    
        sidebar_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        # Bind enter/leave to only scroll when mouse is over sidebar
        def _bind_mousewheel(event):
            sidebar_canvas.bind_all("<MouseWheel>", _on_mousewheel)
    
        def _unbind_mousewheel(event):
            sidebar_canvas.unbind_all("<MouseWheel>")

        sidebar_canvas.bind("<Enter>", _bind_mousewheel)
        sidebar_canvas.bind("<Leave>", _unbind_mousewheel)

        # --- Main canvas ---
        self.canvas = tk.Canvas(root, bg="#ffffff", highlightthickness=0)
        self.canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.editor = PolygonEditor(self.canvas)

        # Controls
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TButton", padding=6)
        style.configure("TLabel", padding=2)

        self.btn_add_mode = ttk.Button(self.sidebar, text="Add Mode (Click)", command=self._set_add_mode)
        self.btn_move_mode = ttk.Button(self.sidebar, text="Move Mode (Drag)", command=self._set_move_mode)
        self.btn_delete_mode = ttk.Button(self.sidebar, text="Delete Mode (Click)", command=self._set_delete_mode)
        self.btn_socket_mode = ttk.Button(self.sidebar, text="Socket Mode (Click Edge)", command=self._set_socket_mode)
        self.btn_toggle_opt = ttk.Button(self.sidebar, text="Show Optimal Only", command=self._toggle_opt)
        self.btn_close = ttk.Button(self.sidebar, text="Close Polygon", command=self._close_poly)
        self.btn_compute = ttk.Button(self.sidebar, text="Compute Guards", command=self._compute)
        self.btn_reset = ttk.Button(self.sidebar, text="Reset", command=self._reset)

        self.btn_save = ttk.Button(self.sidebar, text="Save JSON", command=self._save)
        self.btn_load = ttk.Button(self.sidebar, text="Load JSON", command=self._load)
        self.btn_export = ttk.Button(self.sidebar, text="Export PNG", command=self._export_png)

        for w in [self.btn_add_mode, self.btn_move_mode, self.btn_delete_mode, self.btn_socket_mode, self.btn_toggle_opt, self.btn_close, self.btn_compute, self.btn_reset, self.btn_save, self.btn_load, self.btn_export]:
            w.pack(fill=tk.X, padx=10, pady=6)

        # Cost inputs
        cost_frame = ttk.LabelFrame(self.sidebar, text="Costs")
        cost_frame.pack(fill=tk.X, padx=10, pady=6)
        ttk.Label(cost_frame, text="Wire per unit:").grid(row=0, column=0, sticky="w")
        ttk.Label(cost_frame, text="Camera cost:").grid(row=1, column=0, sticky="w")
        self.var_wire = tk.StringVar(value="1.0")
        self.var_cam = tk.StringVar(value="1.0")
        ttk.Entry(cost_frame, textvariable=self.var_wire, width=10).grid(row=0, column=1, sticky="e")
        ttk.Entry(cost_frame, textvariable=self.var_cam, width=10).grid(row=1, column=1, sticky="e")

        # Stats panel
        stats_frame = ttk.LabelFrame(self.sidebar, text="Statistics")
        stats_frame.pack(fill=tk.X, padx=10, pady=6)
        self.stats_vars: Dict[str, tk.StringVar] = {
            "n": tk.StringVar(value="n: -"),
            "tri": tk.StringVar(value="triangles: -"),
            "guards": tk.StringVar(value="guards: -"),
            "class": tk.StringVar(value="class: -"),
            "wire_len": tk.StringVar(value="wire length: -"),
            "wire_cost": tk.StringVar(value="wire cost: -"),
            "cam_cost": tk.StringVar(value="camera cost: -"),
            "total_cost": tk.StringVar(value="total cost: -"),
        }
        r = 0
        for key in ["n", "tri", "guards", "class", "wire_len", "wire_cost", "cam_cost", "total_cost"]:
            ttk.Label(stats_frame, textvariable=self.stats_vars[key]).grid(row=r, column=0, sticky="w")
            r += 1

        # Class-wise stats (always visible)
        self.class_frame = ttk.LabelFrame(self.sidebar, text="Class-wise costs (0/1/2)")
        self.class_frame.pack(fill=tk.X, padx=10, pady=6)
        self.class_vars: Dict[str, tk.StringVar] = {
            "c0": tk.StringVar(value="0: size -, total -, wire -, cam -"),
            "c1": tk.StringVar(value="1: size -, total -, wire -, cam -"),
            "c2": tk.StringVar(value="2: size -, total -, wire -, cam -"),
        }
        ttk.Label(self.class_frame, textvariable=self.class_vars["c0"]).pack(anchor="w")
        ttk.Label(self.class_frame, textvariable=self.class_vars["c1"]).pack(anchor="w")
        ttk.Label(self.class_frame, textvariable=self.class_vars["c2"]).pack(anchor="w")

        # Replace old freeform status label with concise hint-only label
        self.status_var = tk.StringVar(value="Add vertices → Close → Socket → Costs → Compute")
        # self.status_label = ttk.Label(self.sidebar, textvariable=self.status_var, wraplength=240, justify="left", foreground="#555")
        # self.status_label.pack(fill=tk.X, padx=10, pady=4)

        # Interactions
        self.mode = "add"
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Configure>", lambda e: self.editor._refresh_view())

    # ======= Control callbacks =======
    def _set_add_mode(self):
        self.mode = "add"
        self.status_var.set("Add Mode: click canvas to append vertices.")

    def _set_move_mode(self):
        self.mode = "move"
        self.status_var.set("Move Mode: click and drag a vertex to move it.")

    def _set_delete_mode(self):
        self.mode = "delete"
        self.status_var.set("Delete Mode: click a vertex to remove it.")

    def _set_socket_mode(self):
        self.mode = "socket"
        self.status_var.set("Socket Mode: click near an edge to place the socket.")

    def _toggle_opt(self):
        self.editor.show_opt_only = not self.editor.show_opt_only
        txt = "Show All" if self.editor.show_opt_only else "Show Optimal Only"
        self.btn_toggle_opt.config(text=txt)
        self.editor._refresh_view()

    def _toggle_class_panel(self):
        self.class_panel_visible = not self.class_panel_visible
        if self.class_panel_visible:
            self.class_frame.pack(fill=tk.X, padx=10, pady=6)
            self.btn_toggle_class.config(text="Hide Class-wise Stats")
        else:
            self.class_frame.pack_forget()
            self.btn_toggle_class.config(text="Show Class-wise Stats")

    def _close_poly(self):
        self.editor.close_polygon()
        self.status_var.set("Polygon closed.")

    def _compute(self):
        # Pull costs
        try:
            self.editor.cost_wire_per_unit = float(self.var_wire.get())
            self.editor.cost_camera = float(self.var_cam.get())
        except Exception:
            self.status_var.set("Invalid cost inputs.")
            return
        ok, msg = self.editor.compute()
        self.status_var.set(msg)
        self._refresh_stats()

    def _reset(self):
        self.editor.reset()
        self.status_var.set("Reset. Click to add vertices.")

    def _save(self):
        try:
            self.editor.save_json("polygon.json")
            self.status_var.set("Saved to polygon.json")
        except Exception as e:
            self.status_var.set(f"Save failed: {e}")

    def _load(self):
        try:
            self.editor.load_json("polygon.json")
            self.status_var.set("Loaded polygon.json")
        except Exception as e:
            self.status_var.set(f"Load failed: {e}")
        self._refresh_stats()

    def _export_png(self):
        try:
            # Save PS then try Pillow to convert to PNG
            ps_path = "canvas.ps"
            png_path = "output.png"
            self.canvas.postscript(file=ps_path, colormode='color')
            try:
                from PIL import Image
                img = Image.open(ps_path)
                img.save(png_path, "png")
                self.status_var.set(f"Exported {png_path}")
            except Exception:
                self.status_var.set(f"Saved PostScript {ps_path} (install Pillow for PNG)")
        except Exception as e:
            self.status_var.set(f"Export failed: {e}")

    # ======= Canvas interactions =======
    def _screen_to_world(self, x: int, y: int) -> Point:
        w = int(self.canvas.winfo_width())
        h = int(self.canvas.winfo_height())
        vx = self.editor.viewport.min_x + (x / max(w, 1)) * (self.editor.viewport.max_x - self.editor.viewport.min_x)
        vy = self.editor.viewport.max_y - (y / max(h, 1)) * (self.editor.viewport.max_y - self.editor.viewport.min_y)
        return (vx, vy)

    def _hit_vertex(self, x: int, y: int, radius_px: int = 10) -> Optional[int]:
        # Return index of nearest vertex within radius
        w = int(self.canvas.winfo_width())
        h = int(self.canvas.winfo_height())
        best = None
        best_d2 = (radius_px + 1) ** 2
        for i, p in enumerate(self.editor.points):
            sx, sy = self.editor.viewport.world_to_screen(p, w, h)
            dx = sx - x
            dy = sy - y
            d2 = dx * dx + dy * dy
            if d2 <= best_d2:
                best_d2 = d2
                best = i
        return best

    def _on_click(self, event):
        if self.mode == "add" and not self.editor.closed:
            vx, vy = self._screen_to_world(event.x, event.y)
            self.editor.add_point((vx, vy))
            return
        if self.mode == "move":
            idx = self._hit_vertex(event.x, event.y)
            self.editor.selected_index = idx
            self.editor.drag_index = idx
            self.editor._refresh_view()
            return
        if self.mode == "delete":
            idx = self._hit_vertex(event.x, event.y)
            if idx is not None and not self.editor.closed:
                del self.editor.points[idx]
                # Clear computed data
                self.editor.triangles.clear()
                self.editor.diagonals.clear()
                self.editor.colors.clear()
                self.editor.guards.clear()
                self.editor.selected_index = None
                self.editor._refresh_view()
                return
        if self.mode == "socket" and self.editor.closed:
            # pick nearest edge and parameter t
            n = len(self.editor.points)
            w = int(self.canvas.winfo_width())
            h = int(self.canvas.winfo_height())
            best = None
            best_d2 = 1e18
            best_t = 0.0
            for k in range(n):
                a = self.editor.points[k]
                b = self.editor.points[(k + 1) % n]
                # project click to segment in screen space for more intuitive picking
                ax, ay = self.editor.viewport.world_to_screen(a, w, h)
                bx, by = self.editor.viewport.world_to_screen(b, w, h)
                vx = bx - ax
                vy = by - ay
                seg_len2 = vx * vx + vy * vy
                t = 0.0 if seg_len2 == 0 else max(0.0, min(1.0, ((event.x - ax) * vx + (event.y - ay) * vy) / seg_len2))
                px = ax + t * vx
                py = ay + t * vy
                dx = px - event.x
                dy = py - event.y
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best = k
                    best_t = t
            if best is not None:
                self.editor.socket_edge = best
                self.editor.socket_t = best_t
                # Auto-recompute with current costs
                try:
                    self.editor.cost_wire_per_unit = float(self.var_wire.get())
                    self.editor.cost_camera = float(self.var_cam.get())
                except Exception:
                    pass
                ok, msg = self.editor.compute()
                self.status_var.set(msg if ok else "")
                self._refresh_stats()

    def _on_drag(self, event):
        if self.mode == "move" and self.editor.drag_index is not None and not self.editor.closed:
            vx, vy = self._screen_to_world(event.x, event.y)
            self.editor.points[self.editor.drag_index] = (vx, vy)
            # invalidate computed artifacts while moving
            self.editor.triangles.clear()
            self.editor.diagonals.clear()
            self.editor.colors.clear()
            self.editor.guards.clear()
            self.editor._refresh_view()
            self._refresh_stats()

    def _on_release(self, event):
        if self.mode == "move":
            self.editor.drag_index = None

    def _refresh_stats(self):
        st = getattr(self.editor, "stats", {}) or {}
        def fmt(k, fmt_spec="{:.3f}"):
            v = st.get(k)
            if v is None:
                return "-"
            try:
                if k in ("n", "tri", "guards", "class"):
                    return str(int(v)) if v >= 0 else "-"
                return fmt_spec.format(float(v))
            except Exception:
                return str(v)
        if hasattr(self, "stats_vars"):
            self.stats_vars["n"].set(f"n: {fmt('n','{:.0f}')} ")
            self.stats_vars["tri"].set(f"triangles: {fmt('tri','{:.0f}')} ")
            cls = fmt('class','{:.0f}')
            self.stats_vars["class"].set(f"class: {cls}")
            self.stats_vars["guards"].set(f"guards: {fmt('guards','{:.0f}')} ")
            self.stats_vars["wire_len"].set(f"wire length: {fmt('wire_len')} ")
            self.stats_vars["wire_cost"].set(f"wire cost: {fmt('wire_cost')} ")
            self.stats_vars["cam_cost"].set(f"camera cost: {fmt('cam_cost')} ")
            self.stats_vars["total_cost"].set(f"total cost: {fmt('total_cost')} ")
        # Update class-wise lines
        cs = getattr(self.editor, "class_stats", {}) or {}
        def cf(cid: int, key: str, spec="{:.3f}"):
            row = cs.get(cid)
            if not row:
                return "-"
            if key == "size":
                return str(int(row.get(key, 0)))
            try:
                return spec.format(float(row.get(key, 0.0)))
            except Exception:
                return str(row.get(key, "-"))
        if hasattr(self, "class_vars"):
            self.class_vars["c0"].set(f"0: size {cf(0,'size','{:.0f}')}, total {cf(0,'total_cost')}, wire {cf(0,'wire_cost')}, cam {cf(0,'cam_cost')}")
            self.class_vars["c1"].set(f"1: size {cf(1,'size','{:.0f}')}, total {cf(1,'total_cost')}, wire {cf(1,'wire_cost')}, cam {cf(1,'cam_cost')}")
            self.class_vars["c2"].set(f"2: size {cf(2,'size','{:.0f}')}, total {cf(2,'total_cost')}, wire {cf(2,'wire_cost')}, cam {cf(2,'cam_cost')}")


def main():
    root = tk.Tk()
    app = ArtGalleryApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()


