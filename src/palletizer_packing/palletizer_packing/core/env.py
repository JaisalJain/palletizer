"""Gymnasium environment over the same pallet state, rules and physics checker.

Observation: {'heightmap': (H, W) float32 in [0, max_height],
              'box': [length, width, height, mass] float32,
              'lookahead': (k, 4) float32 (next boxes, zero-padded)}
Action: Discrete(n_u * n_v * 2) -> footprint min corner on the candidate grid x orientation.
Reward: placed box volume as % of the usable pallet volume; `invalid_penalty` for an
infeasible action (the same box is offered again). The episode ends when no feasible
placement exists for the current box (pallet full), after `max_invalid` invalid actions in
a row, or when the sequence runs out. `action_masks()` returns the feasible actions
(for masked policies). Nothing is trained here.
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from palletizer_packing.core.pallet import PalletState
from palletizer_packing.core.planners import make_placement
from palletizer_packing.core.rules import feasible
from palletizer_packing.core.sequence import box_sequence


class PalletizingEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, spec, rules, spawner_cfg, reach_map=None, physics=None, step=0.05,
                 n_boxes=100, lookahead=2, invalid_penalty=-1.0, max_invalid=20):
        super().__init__()
        self.pallet_spec, self.rules, self.spawner_cfg = spec, rules, spawner_cfg
        self.reach, self.physics = reach_map, physics
        self.n_boxes, self.k = n_boxes, lookahead
        self.invalid_penalty, self.max_invalid = invalid_penalty, max_invalid
        self.us = np.round(np.arange(0.0, spec.length + 1e-9, step), 4)
        self.vs = np.round(np.arange(0.0, spec.width + 1e-9, step), 4)
        self.action_space = spaces.Discrete(len(self.us) * len(self.vs) * 2)
        self.observation_space = spaces.Dict({
            'heightmap': spaces.Box(0.0, spec.max_height, spec.shape, dtype=np.float32),
            'box': spaces.Box(0.0, np.inf, (4,), dtype=np.float32),
            'lookahead': spaces.Box(0.0, np.inf, (lookahead, 4), dtype=np.float32),
        })
        self.state = None

    # --------------------------------------------------------------- helpers
    def decode(self, a):
        rot = a % 2
        a //= 2
        iu, iv = a % len(self.us), a // len(self.us)
        return float(self.us[iu]), float(self.vs[iv]), bool(rot)

    def _placement(self, a):
        u0, v0, rot = self.decode(int(a))
        box = self.box
        du, dv = (box.width, box.length) if rot else (box.length, box.width)
        return make_placement(self.state, box, u0, v0, du, dv, rot)

    def _feasible(self, p):
        ok, reason = feasible(self.state, p, self.box, self.rules, self.reach)
        if ok and self.physics is not None:
            ok = self.physics.stable(self.state, p, self.box)
            reason = '' if ok else 'physics'
        return ok, reason

    def action_masks(self):
        m = np.zeros(self.action_space.n, dtype=bool)
        for a in range(self.action_space.n):
            ok, _ = feasible(self.state, self._placement(a), self.box, self.rules, self.reach)
            m[a] = ok
        return m

    def _obs(self):
        la = np.zeros((self.k, 4), dtype=np.float32)
        for i, b in enumerate(self.seq[self.i + 1:self.i + 1 + self.k]):
            la[i] = (b.length, b.width, b.height, b.mass)
        b = self.box
        return {'heightmap': self.state.height.astype(np.float32),
                'box': np.array([b.length, b.width, b.height, b.mass], dtype=np.float32),
                'lookahead': la}

    @property
    def box(self):
        return self.seq[self.i]

    # --------------------------------------------------------------- gym API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        seq_seed = int(self.np_random.integers(0, 2**31 - 1)) if seed is None else seed
        self.seq = box_sequence(self.spawner_cfg, seq_seed, self.n_boxes)
        self.i = 0
        self.invalid = 0
        self.state = PalletState(self.pallet_spec)
        return self._obs(), {'fill_percent': 0.0}

    def step(self, action):
        p = self._placement(action)
        ok, reason = self._feasible(p)
        info = {'reason': reason, 'placement': p}
        if not ok:
            self.invalid += 1
            terminated = not self.action_masks().any()
            truncated = self.invalid >= self.max_invalid
            info['fill_percent'] = self.state.fill_percent()
            return self._obs(), self.invalid_penalty, terminated, truncated, info
        self.invalid = 0
        self.state.place(p, self.box)
        ps = self.pallet_spec
        reward = 100.0 * self.box.volume / (ps.length * ps.width * ps.max_height)
        self.i += 1
        info['fill_percent'] = self.state.fill_percent()
        if self.i >= len(self.seq):
            return self._final_obs(), reward, False, True, info
        terminated = not self.action_masks().any()
        return self._obs(), reward, terminated, False, info

    def _final_obs(self):
        la = np.zeros((self.k, 4), dtype=np.float32)
        return {'heightmap': self.state.height.astype(np.float32),
                'box': np.zeros(4, dtype=np.float32), 'lookahead': la}
