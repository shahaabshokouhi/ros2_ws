# Copied from ma_explore (2f60eaf) by scripts/sync_robot_package.sh: edit it there, not here.
"""Load a trained actor for deployment, and export the slim file the robots use.

A robot needs only the actor and the environment settings it was trained
with (the observation is built from them), not the critic or the optimizer
state. export_robot_policy() writes that file next to the checkpoints
(robot_policy.pt); load_policy_file() reads it or a full checkpoint, on any
device, without the training code (no MAPPO, no maps, no compile).

    python -m ma_explore.policy_io RUN/checkpoints/best.pt [OUT.pt]   # export
"""
import datetime
import os
import sys

import torch

from .env import EnvConfig, obs_dims
from .model import Actor

# the network sizes of the PPO config (model.Actor arguments)
NET_KEYS = {'d_model': 128, 'layers': 2, 'heads': 4, 'width': 64}


def build_actor(config, device='cpu'):
    env_cfg = EnvConfig(**config['env'])
    net = {k: config.get('ppo', {}).get(k, v) for k, v in NET_KEYS.items()}
    actor = Actor(net['d_model'], net['layers'], net['heads'], net['width'], obs_dims(env_cfg)).to(device)
    return actor, env_cfg


def load_policy_file(path, device='cpu'):
    """(policy(obs) -> greedy actions (B, N), EnvConfig, map size, meta) from a
    robot_policy.pt or a full training checkpoint."""
    ck = torch.load(os.path.expanduser(path), map_location=device, weights_only=False)
    actor, env_cfg = build_actor(ck['config'], device)
    actor.load_state_dict(ck['actor'])
    actor.eval()
    keys = ('planes', 'cand', 'cand_mask', 'cand_cells', 'mates', 'scalars')

    @torch.no_grad()
    def policy(obs):
        B, N = obs['planes'].shape[:2]
        flat = [obs[k].flatten(0, 1) for k in keys]
        flat[3] = flat[3].long()
        return actor(*flat).argmax(-1).view(B, N)

    meta = ck.get('meta', {'run': '?', 'update': ck.get('update'), 'env_steps': ck.get('env_steps')})
    return policy, env_cfg, int(ck['config']['train']['map_size']), meta


def export_robot_policy(checkpoint, out=None):
    """Write the actor + settings of `checkpoint` to `out` (default: robot_policy.pt beside it)."""
    ck = torch.load(checkpoint, map_location='cpu', weights_only=False)
    run = os.path.dirname(os.path.dirname(os.path.abspath(checkpoint)))
    out = out or os.path.join(os.path.dirname(os.path.abspath(checkpoint)), 'robot_policy.pt')
    meta = {'run': os.path.basename(run), 'checkpoint': os.path.basename(checkpoint), 'update': ck.get('update'),
            'env_steps': ck.get('env_steps'), 'score': ck.get('best'),
            'exported': datetime.datetime.now().isoformat(timespec='seconds')}
    torch.save({'actor': ck['actor'], 'config': ck['config'], 'meta': meta}, out)
    return out, meta


if __name__ == '__main__':
    p, m = export_robot_policy(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    print(f'{p}: {m}')
