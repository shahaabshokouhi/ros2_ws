# Copied from ma_explore (485a9b8) by scripts/sync_robot_package.sh: edit it there, not here.
"""Classic decentralized exploration policies, on the same observations as the
learned policy (each robot uses only its own integrated map and what it knows
about merged teammates). All commit to a goal until it is reached.

  nearest      nearest frontier candidate by path distance
  utility      information gain minus beta x path cost (cost-utility frontier)
  coordinated  utility, minus a penalty near merged teammates' goals
               (decentralized version of Burgard et al., T-RO 2005)
  slam_rules   hand-designed SLAM-aware rules on the stage-2 observation (the
               baseline a learned SLAM-aware policy must beat): WAIT while the
               SLAM backlog is over wait_backlog s; on choosing a new goal after
               more than revisit_after m without a map correction, the revisit
               pose with the most gain (connectivity + merge accuracy) within
               revisit_max_m; with own-pose slots, when the pose uncertainty is
               over loop_unc (graph resistance, in nodes), the own pose whose loop
               removes the most within revisit_max_m; otherwise coordinated.
               Stage 1: coordinated.

Candidate features (env.observe): 0-1 relative position, 2 path distance,
3 straight distance (all / (H + W)), 4 information gain (fraction of the sensor
disk unknown), 5 is the current goal, 6 distance to the nearest known
teammate goal, 7 distance to the nearest known teammate.
"""
import torch


def _commit(obs, choice):
    """Keep the current goal while there is one (candidate 0 flagged as goal)."""
    has_goal = obs['cand'][..., 0, 5] > 0
    return torch.where(has_goal, torch.zeros_like(choice), choice)


def _frontier_mask(obs):
    """Frontier candidates only (not keep, nor stage 2's wait and revisit slots)."""
    m = obs['cand_mask'].clone()
    m[..., 0] = False
    f = obs['cand']
    if f.shape[-1] > 8:                      # stage 2: features 8 (wait) and 9 (revisit)
        m &= (f[..., 8] == 0) & (f[..., 9] == 0)
    if f.shape[-1] >= 17:                    # own-pose revisit slots (feature 15)
        m &= f[..., 15] == 0
    return m


def _argmax_or_stay(score, mask):
    s = torch.where(mask, score, torch.full_like(score, -1e9))
    a = s.argmax(-1)
    return torch.where(mask.any(-1), a, torch.zeros_like(a))


def nearest(obs):
    f = obs['cand']
    return _commit(obs, _argmax_or_stay(-f[..., 2], _frontier_mask(obs)))


def utility(obs, beta=4.0):
    f = obs['cand']
    return _commit(obs, _argmax_or_stay(f[..., 4] - beta * f[..., 2], _frontier_mask(obs)))


def coordinated(obs, beta=4.0, alpha=1.0, sigma=0.08):
    f = obs['cand']
    penalty = alpha * torch.exp(-f[..., 6] / sigma)
    return _commit(obs, _argmax_or_stay(f[..., 4] - beta * f[..., 2] - penalty, _frontier_mask(obs)))


def slam_rules(obs, wait_backlog=1.5, revisit_after=4.0, revisit_max_m=2.0, beta=4.0, loop_unc=3.0):
    f, sc, m = obs['cand'], obs['scalars'], obs['cand_mask']
    a = coordinated(obs)
    if f.shape[-1] <= 8:                     # stage 1 observation: no SLAM state
        return a
    far = 2 * obs['planes'].shape[-1]        # H + W: the path feature's scale
    backlog, since = 3.0 * sc[..., 4], 5.0 * sc[..., 7]
    has_goal = f[..., 0, 5] > 0
    # 1. SLAM behind: wait (the goal is kept)
    wait_ok = m & (f[..., 8] > 0)
    wait = wait_ok.any(-1) & (backlog > wait_backlog)
    # 2. far since the last map correction, choosing anew: the best revisit pose nearby
    rev = m & (f[..., 9] > 0) & (f[..., 2] * far * 0.1 <= revisit_max_m)
    gain = (f[..., 13] + f[..., 14]) if f.shape[-1] >= 15 else torch.ones_like(f[..., 2])
    r = _argmax_or_stay(gain - beta * f[..., 2], rev)
    go_rev = rev.any(-1) & ~has_goal & (since > revisit_after)
    a = torch.where(go_rev, r, a)
    if f.shape[-1] >= 17:                    # own poses: close a loop when the pose uncertainty is high
        unc = 10.0 * sc[..., -1]
        own = m & (f[..., 15] > 0) & (f[..., 2] * far * 0.1 <= revisit_max_m)
        o = _argmax_or_stay(f[..., 16] - beta * f[..., 2], own)
        a = torch.where(own.any(-1) & ~has_goal & (unc > loop_unc), o, a)
    return torch.where(wait, wait_ok.float().argmax(-1), a)


def random_policy(obs, gen=None):
    m = obs['cand_mask'].float()
    a = torch.multinomial(m.view(-1, m.shape[-1]), 1, generator=gen).view(m.shape[:2])
    return a


BASELINES = {'nearest': nearest, 'utility': utility, 'coordinated': coordinated, 'slam_rules': slam_rules,
             'random': random_policy}
