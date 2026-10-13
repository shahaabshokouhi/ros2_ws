# Copied from ma_explore (3c9821f) by scripts/sync_robot_package.sh: edit it there, not here.
"""Actor (decentralized: one robot's own observation) and critic (centralized:
global state, used in training only).

Actor: a CNN encodes the robot's map planes; tokens for the robot itself
(global map feature + scalars), each teammate (known only once merged;
others are masked out) and each candidate goal (its features + the map
feature under it) go through a small transformer; each candidate token gives
one logit (a pointer network over the candidates). Invalid candidates are masked.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

from .env import STAGE1_DIMS


def init_(m, gain=1.0):
    if isinstance(m, (nn.Linear, nn.Conv2d)):
        nn.init.orthogonal_(m.weight, gain)
        nn.init.zeros_(m.bias)
    return m


class MapEncoder(nn.Module):
    """CNN: (C, H, W) -> feature maps at 1/4 and 1/8 size, and a global vector.
    The input is first folded 2x2 into channels (space-to-depth, lossless), so
    the first layer works at half resolution: about 4x cheaper."""

    def __init__(self, c_in, width=64, d=128):
        super().__init__()
        self.c1 = nn.Conv2d(4 * c_in, width // 2, 3, 1, 1)
        self.c2 = nn.Conv2d(width // 2, width, 3, 2, 1)
        self.c3 = nn.Conv2d(width, 2 * width, 3, 2, 1)
        self.c4 = nn.Conv2d(2 * width, 2 * width, 3, 2, 1)
        self.n1, self.n2 = nn.GroupNorm(8, width // 2), nn.GroupNorm(8, width)
        self.n3, self.n4 = nn.GroupNorm(8, 2 * width), nn.GroupNorm(8, 2 * width)
        self.out = nn.Linear(2 * width, d)
        for m in self.modules():
            init_(m, nn.init.calculate_gain('relu'))
        self.out_dim, self.local_dim = d, 3 * width

    def forward(self, x):
        x = F.gelu(self.n1(self.c1(F.pixel_unshuffle(x, 2))))
        f2 = F.gelu(self.n2(self.c2(x)))            # 1/4
        f3 = F.gelu(self.n3(self.c3(f2)))           # 1/8
        f4 = F.gelu(self.n4(self.c4(f3)))           # 1/16
        g = self.out(f4.mean((2, 3)))
        return f2, f3, g


def sample_at(fmap, cells, H, W):
    """Bilinear features of fmap (B, C, h, w) at cells (B, K, 2) given in (H, W) grid cells."""
    y = (cells[..., 0].float() + 0.5) / H * 2 - 1
    x = (cells[..., 1].float() + 0.5) / W * 2 - 1
    grid = torch.stack([x, y], -1)[:, :, None]               # (B, K, 1, 2)
    return F.grid_sample(fmap, grid, align_corners=False)[..., 0].transpose(1, 2)   # (B, K, C)


class Actor(nn.Module):
    def __init__(self, d=128, layers=2, heads=4, width=64, dims=None):
        super().__init__()
        D = dims or STAGE1_DIMS          # input sizes (env.obs_dims): stage 1 unless given
        self.enc = MapEncoder(D['planes'], width, d)
        self.self_tok = nn.Sequential(init_(nn.Linear(d + D['scalars'], d)), nn.GELU(), init_(nn.Linear(d, d)))
        self.mate_tok = nn.Sequential(init_(nn.Linear(D['mate'], d)), nn.GELU(), init_(nn.Linear(d, d)))
        self.cand_tok = nn.Sequential(init_(nn.Linear(D['cand'] + self.enc.local_dim, d)), nn.GELU(),
                                      init_(nn.Linear(d, d)))
        self.type_emb = nn.Parameter(torch.zeros(3, d))
        layer = nn.TransformerEncoderLayer(d, heads, 2 * d, dropout=0.0, activation='gelu',
                                           batch_first=True, norm_first=True)
        self.tf = nn.TransformerEncoder(layer, layers, enable_nested_tensor=False)
        self.norm = nn.LayerNorm(d)
        self.head = init_(nn.Linear(d, 1), 0.01)

    def forward(self, planes, cand, cand_mask, cand_cells, mates, scalars):
        """All inputs flattened over (env, robot): planes (M, C, H, W) uint8,
        cand (M, K, F), cand_mask (M, K), cand_cells (M, K, 2), mates (M, N-1, Fm),
        scalars (M, S). Returns masked logits (M, K)."""
        M, _, H, W = planes.shape
        f2, f3, g = self.enc(planes.float())
        local = torch.cat([sample_at(f2, cand_cells, H, W), sample_at(f3, cand_cells, H, W)], -1)
        t_self = self.self_tok(torch.cat([g, scalars], -1))[:, None] + self.type_emb[0]
        t_mate = self.mate_tok(mates) + self.type_emb[1]
        t_cand = self.cand_tok(torch.cat([cand, local], -1)) + self.type_emb[2]
        x = torch.cat([t_self, t_mate, t_cand], 1)
        pad = torch.cat([torch.zeros(M, 1, dtype=torch.bool, device=x.device), mates[..., 0] < 0.5, ~cand_mask], 1)
        x = self.tf(x, src_key_padding_mask=pad)
        logits = self.head(self.norm(x[:, 1 + mates.shape[1]:]))[..., 0].float()
        return logits.masked_fill(~cand_mask, -1e9)


class Critic(nn.Module):
    def __init__(self, d=128, width=64, dims=None):
        super().__init__()
        D = dims or STAGE1_DIMS
        self.enc = MapEncoder(D['state_planes'], width, d)
        self.mlp = nn.Sequential(init_(nn.Linear(d + D['state_scalars'], 2 * d)), nn.GELU(),
                                 init_(nn.Linear(2 * d, d)), nn.GELU(), init_(nn.Linear(d, 1), 1.0))

    def forward(self, state, state_scalars):
        """state (M, 6, H, W): the environment's 5 planes + the robot's own position plane."""
        _, _, g = self.enc(state.float())
        return self.mlp(torch.cat([g, state_scalars], -1))[..., 0].float()
