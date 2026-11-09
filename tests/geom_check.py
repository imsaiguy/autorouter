"""Independent, exact-geometry checks of a routing result (no grid involved)."""
import math

from kicad_autorouter.problem import seg_point_dist


def seg_seg_dist(a, b):
    (ax1, ay1, ax2, ay2), (bx1, by1, bx2, by2) = a, b

    def ccw(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    d1 = ccw(bx1, by1, bx2, by2, ax1, ay1)
    d2 = ccw(bx1, by1, bx2, by2, ax2, ay2)
    d3 = ccw(ax1, ay1, ax2, ay2, bx1, by1)
    d4 = ccw(ax1, ay1, ax2, ay2, bx2, by2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 and d2 and d3 and d4:
        return 0.0
    return min(seg_point_dist(ax1, ay1, bx1, by1, bx2, by2), seg_point_dist(ax2, ay2, bx1, by1, bx2, by2),
               seg_point_dist(bx1, by1, ax1, ay1, ax2, ay2), seg_point_dist(bx2, by2, ax1, ay1, ax2, ay2))


def clearance_violations(board, result, s, tol=1e-3):
    """List of human-readable violations between copper of different nets.
    Uses each segment's own width and the larger of the two nets' clearances."""
    w0 = s.track_width
    segs = [(n, sg, nr.width(k, w0)) for n, nr in result.nets.items() for k, sg in enumerate(nr.segments)]
    vias = [(n, v) for n, nr in result.nets.items() for v in nr.vias]
    cl = s.clearance_for
    out = []
    # segment - segment
    for i in range(len(segs)):
        ni, a, wa = segs[i]
        for j in range(i + 1, len(segs)):
            nj, b, wb = segs[j]
            if ni == nj or a[4] != b[4]:
                continue
            d = seg_seg_dist(a[:4], b[:4]) - (wa + wb) / 2
            if d < max(cl(ni), cl(nj)) - tol:
                out.append(f"track {ni} vs track {nj} on {a[4]}: {d:.3f} mm")
    # via - segment, via - via
    for nv, (vx, vy) in vias:
        for ns, sg, ws in segs:
            if nv != ns:
                d = seg_point_dist(vx, vy, *sg[:4]) - ws / 2 - s.via_diameter / 2
                if d < max(cl(nv), cl(ns)) - tol:
                    out.append(f"via {nv} vs track {ns}: {d:.3f} mm")
        for nv2, (x2, y2) in vias:
            if nv2 != nv and (x2, y2) > (vx, vy):
                d = math.hypot(x2 - vx, y2 - vy) - s.via_diameter
                if d < max(cl(nv), cl(nv2)) - tol:
                    out.append(f"via {nv} vs via {nv2}: {d:.3f} mm")
    # copper - pads of other nets
    for p in board.pads:
        for ns, sg, ws in segs:
            if ns != p.net and sg[4] in p.layers:
                # sample along the segment (pads are not segments)
                L = math.hypot(sg[2] - sg[0], sg[3] - sg[1])
                n = max(2, int(L / 0.02))
                d = min(p.distance(sg[0] + (sg[2] - sg[0]) * k / n, sg[1] + (sg[3] - sg[1]) * k / n)
                        for k in range(n + 1)) - ws / 2
                if d < max(cl(ns), cl(p.net)) - 0.01:
                    out.append(f"track {ns} vs pad {p.ref} ({p.net}): {d:.3f} mm")
        for nv, (vx, vy) in vias:
            if nv != p.net:
                d = p.distance(vx, vy) - s.via_diameter / 2
                if d < max(cl(nv), cl(p.net)) - tol:
                    out.append(f"via {nv} vs pad {p.ref} ({p.net}): {d:.3f} mm")
    return out


def connected(board, result, net, w):
    """True if all pads of `net` are joined by the routed copper."""
    pads = [p for p in board.pads if p.net == net]
    nr = result.nets[net]
    items = [("p", p) for p in pads] + [("s", s) for s in nr.segments] + [("v", v) for v in nr.vias]
    parent = list(range(len(items)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def touch(a, b):
        ka, A = a
        kb, B = b
        if ka == "s" and kb == "s":
            return A[4] == B[4] and seg_seg_dist(A[:4], B[:4]) < 1e-6
        if {ka, kb} == {"s", "p"}:
            S, P = (A, B) if ka == "s" else (B, A)
            return S[4] in P.layers and min(P.distance(S[0], S[1]), P.distance(S[2], S[3])) <= 1e-6
        if {ka, kb} == {"s", "v"}:
            S, V = (A, B) if ka == "s" else (B, A)
            return seg_point_dist(V[0], V[1], *S[:4]) < 1e-6   # vias may sit mid-track (T-junction)
        if ka == "p" and kb == "p":
            return False
        if {ka, kb} == {"v", "p"}:
            V, P = (A, B) if ka == "v" else (B, A)
            return P.distance(*V) <= 1e-6
        return False
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            if touch(items[i], items[j]):
                parent[find(i)] = find(j)
    roots = {find(i) for i in range(len(pads))}
    return len(roots) == 1


def dangling_ends(board, result):
    """Track ends that touch nothing: no pad, via, other track of the net, or existing copper."""
    out = []
    for name, nr in result.nets.items():
        pads = [p for p in board.pads if p.net == name]
        old = [t for t in board.tracks if t.net == name]
        oldv = [v for v in board.vias if v.net == name]
        segs = nr.segments
        for i, s in enumerate(segs):
            for (x, y) in ((s[0], s[1]), (s[2], s[3])):
                ok = (any(s[4] in p.layers and p.distance(x, y) <= 1e-6 for p in pads)
                      or any(math.hypot(x - vx, y - vy) < 1e-6 for vx, vy in nr.vias)
                      or any(math.hypot(x - v.x, y - v.y) < 1e-6 for v in oldv)
                      or any(j != i and t[4] == s[4] and seg_point_dist(x, y, *t[:4]) < 1e-6
                             for j, t in enumerate(segs))
                      or any(t.layer == s[4] and seg_point_dist(x, y, t.x1, t.y1, t.x2, t.y2) <= t.width / 2
                             for t in old))
                if not ok:
                    out.append((name, (round(x, 3), round(y, 3)), s[4]))
    return out


def sharp_corners(board, result, limit_deg=46.0):
    """Corners sharper than 45 degrees where exactly two tracks meet (not inside a pad,
    not at a via - those are junction points where any angle is normal)."""
    out = []
    for name, nr in result.nets.items():
        segs = nr.segments
        ends = {}
        for i, s in enumerate(segs):
            ends.setdefault((round(s[0], 4), round(s[1], 4), s[4]), []).append((i, 0))
            ends.setdefault((round(s[2], 4), round(s[3], 4), s[4]), []).append((i, 1))
        vias = {(round(x, 4), round(y, 4)) for x, y in nr.vias}
        for (x, y, layer), lst in ends.items():
            if len(lst) != 2 or (x, y) in vias:
                continue
            if any(p.net == name and layer in p.layers and p.distance(x, y) <= 0 for p in board.pads):
                continue
            (i, ei), (j, ej) = lst
            a, b = segs[i], segs[j]
            va = (a[2] - a[0], a[3] - a[1]) if ei == 1 else (a[0] - a[2], a[1] - a[3])
            vb = (b[2] - b[0], b[3] - b[1]) if ej == 0 else (b[0] - b[2], b[1] - b[3])
            la, lb = math.hypot(*va), math.hypot(*vb)
            if la < 1e-6 or lb < 1e-6:
                continue
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (va[0] * vb[0] + va[1] * vb[1]) / (la * lb)))))
            if ang > limit_deg:
                out.append((name, (x, y), layer, round(ang)))
    return out
