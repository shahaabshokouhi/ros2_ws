# Copied from ma_explore (3605e76) by scripts/sync_robot_package.sh: edit it there, not here.
"""Batched grid geometry on the GPU: sensing, path distances, frontiers, candidate goals.

Everything works on stacks of maps (first dimension = robot in some
environment), so hundreds of environments run as a few tensor operations.
Coordinates are (row, col) = (y, x) cell indices.
"""
import math

import torch
import torch.nn.functional as F

INF = 1.0e4
SQRT2 = math.sqrt(2.0)
_SHIFTS = [(-1, -1, SQRT2), (-1, 0, 1.0), (-1, 1, SQRT2), (0, -1, 1.0),
           (0, 1, 1.0), (1, -1, SQRT2), (1, 0, 1.0), (1, 1, SQRT2)]


# ── sensing ────────────────────────────────────────────────────────────────
def ray_table(radius, n_rays, device):
    """Cell offsets along n_rays rays out to `radius` cells: (n_rays, L, 2)
    int (dy, dx), plus each ray's angle. Rays step half a cell; a ray's
    repeated cells are harmless."""
    L = int(math.ceil(radius * 2))
    ang = torch.arange(n_rays, device=device, dtype=torch.float32) * (2 * math.pi / n_rays)
    r = (torch.arange(1, L + 1, device=device, dtype=torch.float32) * 0.5)
    dy = torch.round(torch.sin(ang)[:, None] * r[None, :]).long()
    dx = torch.round(torch.cos(ang)[:, None] * r[None, :]).long()
    return torch.stack([dy, dx], -1), ang


def sense(occ, pos, rays, ray_ang, heading=None, fov=2 * math.pi):
    """What each robot sees from `pos`.

    occ: (B, H, W) bool true obstacles, one map per robot (expand per agent).
    pos: (B, 2) long cells. Returns flat cell indices into (B, H, W) of the
    free cells and of the obstacle cells seen (each a 1-D LongTensor; repeats
    allowed). A ray stops at the first obstacle (which is seen); cells off the
    map count as obstacles. With `fov` < 2 pi only rays within fov/2 of
    `heading` (B,) radians are used.
    """
    B, H, W = occ.shape
    y = pos[:, 0, None, None] + rays[None, :, :, 0]
    x = pos[:, 1, None, None] + rays[None, :, :, 1]
    inside = (y >= 0) & (y < H) & (x >= 0) & (x < W)
    yc, xc = y.clamp(0, H - 1), x.clamp(0, W - 1)
    b = torch.arange(B, device=occ.device)[:, None, None].expand_as(yc)
    hit = occ[b, yc, xc] | ~inside
    clear = torch.cumprod((~hit).to(torch.uint8), dim=2)
    vis = torch.cat([torch.ones_like(clear[:, :, :1]), clear[:, :, :-1]], dim=2).bool() & inside
    if fov < 2 * math.pi - 1e-6:
        d = torch.remainder(ray_ang[None, :] - heading[:, None] + math.pi, 2 * math.pi) - math.pi
        vis &= (d.abs() <= fov / 2)[:, :, None]
    flat = (b * H + yc) * W + xc
    return flat[vis & ~hit], flat[vis & hit]


# ── path distances ─────────────────────────────────────────────────────────
def _relax(D, block):
    """One octile relaxation step with three pooling ops: the 4-neighbour
    minimum (+1) and the 8-neighbour minimum (+sqrt 2)."""
    n = -D[:, None]
    m4 = -torch.maximum(F.max_pool2d(n, (3, 1), 1, (1, 0)), F.max_pool2d(n, (1, 3), 1, (0, 1)))[:, 0]
    m8 = -F.max_pool2d(n, 3, 1, 1)[:, 0]
    N = torch.minimum(D, torch.minimum(m4 + 1.0, m8 + SQRT2))
    return torch.where(block, torch.full((), INF, device=D.device), N)


_RELAX = None


def _compiled_relax():
    """_relax fused into one GPU kernel by torch.compile (about 9x faster);
    MA_EXPLORE_NO_COMPILE=1 uses the plain version."""
    global _RELAX
    if _RELAX is None:
        import os
        _RELAX = _relax if os.environ.get('MA_EXPLORE_NO_COMPILE') else torch.compile(_relax, dynamic=True)
    return _RELAX


def distance_field(passable, sources, max_iter=None, check_every=8):
    """8-connected shortest-path distance (cells; diagonals sqrt 2) from the
    `sources` cells over `passable` cells, for a stack of maps (B, H, W).
    Unreachable cells (and impassable ones) get INF. Sources are always 0.
    Wavefront relaxation; stops when nothing changes (checked every few steps)."""
    B, H, W = passable.shape
    D = torch.where(sources, torch.zeros((), device=passable.device),
                    torch.full((), INF, device=passable.device)).float()
    block = (~passable) & (~sources)
    max_iter = max_iter or 3 * (H + W)
    relax = _compiled_relax()
    for it in range(0, max_iter, check_every):
        prev = D
        for _ in range(check_every):
            D = relax(D, block)
        if torch.equal(D, prev):
            break
    return D


def inflate(occ, radius=1):
    """Obstacles grown by `radius` cells (square), for where a robot may stand."""
    k = 2 * radius + 1
    return F.max_pool2d(occ.float()[:, None], k, stride=1, padding=radius)[:, 0] > 0


# ── frontiers and candidate goals ──────────────────────────────────────────
def disk_kernel(radius, device):
    r = int(math.ceil(radius))
    yy, xx = torch.meshgrid(torch.arange(-r, r + 1, device=device), torch.arange(-r, r + 1, device=device), indexing='ij')
    return ((yy ** 2 + xx ** 2) <= radius ** 2).float()


def frontiers(free, unknown, stand):
    """Known-free cells next to unknown ones where a robot can stand: (B, H, W) bool."""
    near_unknown = F.max_pool2d(unknown.float()[:, None], 3, stride=1, padding=1)[:, 0] > 0
    return free & near_unknown & stand


def info_gain(unknown, kernel):
    """Unknown cells within the sensor disk around each cell (ignores walls)."""
    r = kernel.shape[0] // 2
    return F.conv2d(unknown.float()[:, None], kernel[None, None], padding=r)[:, 0]


def top_candidates(score, k, nms_radius):
    """Up to k spread-out cells with the highest positive score: (B, k, 2)
    long cells and (B, k) bool valid. Non-maximum suppression keeps one cell
    per (2 nms_radius + 1)^2 window."""
    B, H, W = score.shape
    w = 2 * nms_radius + 1
    peak = F.max_pool2d(score[:, None], w, stride=1, padding=nms_radius)[:, 0]
    s = torch.where((score >= peak) & (score > 0), score, torch.zeros((), device=score.device))
    val, idx = torch.topk(s.view(B, -1), k, dim=1)
    cells = torch.stack([idx // W, idx % W], -1)
    return cells, val > 0
