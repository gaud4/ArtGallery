from typing import List, Tuple, Optional, Dict, Set

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
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            b1, b2 = poly[j], poly[(j + 1) % n]
            if segments_intersect(a1, a2, b1, b2, proper=True):
                return False
    return True

def is_clockwise(poly: List[Point]) -> bool:
    area2 = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        area2 += x1 * y2 - x2 * y1
    return area2 < 0

def point_in_triangle(p: Point, a: Point, b: Point, c: Point) -> bool:
    o1 = orientation(a, b, p)
    o2 = orientation(b, c, p)
    o3 = orientation(c, a, p)
    has_neg = (o1 < -1e-12) or (o2 < -1e-12) or (o3 < -1e-12)
    has_pos = (o1 > 1e-12) or (o2 > 1e-12) or (o3 > 1e-12)
    return not (has_neg and has_pos)

def is_convex(prev: Point, curr: Point, nxt: Point, ccw: bool) -> bool:
    turn = orientation(prev, curr, nxt)
    return turn > 1e-12 if ccw else turn < -1e-12

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

    return triangles, diagonals

def three_color_from_triangulation(poly: List[Point], triangles: List[Tuple[int, int, int]]) -> Dict[int, int]:
    """Color vertices 0,1,2 so every triangle has all three colors."""
    n = len(poly)
    if n < 3:
        return {}

    # Build ear-removal order by reversing triangulation fan
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
