import math
from typing import List, Tuple, Dict
from geometry import Point

def calculate_edge_lengths(points: List[Point]) -> Tuple[List[float], List[float]]:
    """Return edge lengths and prefix sums along boundary starting at vertex 0."""
    n = len(points)
    edges = []
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        edges.append(math.hypot(dx, dy))
    prefix = [0.0]
    for L in edges:
        prefix.append(prefix[-1] + L)
    return edges, prefix  # prefix length n+1 (last = perimeter)

def format_stats(stats: Dict[str, float], format_spec: str = "{:.3f}") -> str:
    """Format statistics with appropriate precision."""
    if not stats:
        return "-"
    try:
        if isinstance(stats, (int, float)):
            return format_spec.format(float(stats))
        return str(stats)
    except Exception:
        return "-"
