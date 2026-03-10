"""Minimal ActorMLP for loading rsl_rl ActorCritic checkpoints.

Reimplements only the actor (policy) network from rsl_rl's ActorCritic,
since rsl_rl is not available in the pixi ROS 2 environment.

Reference: rsl_rl/modules/actor_critic.py (ETH Zurich / NVIDIA)
"""

from __future__ import annotations

from pathlib import Path

import torch
import torch.nn as nn


class ActorMLP(nn.Module):
    """MLP actor network matching rsl_rl's ActorCritic.actor structure.

    Architecture for droid-walking-omni:
      Linear(50, 512) -> ELU -> Linear(512, 256) -> ELU ->
      Linear(256, 128) -> ELU -> Linear(128, 10)
    """

    def __init__(
        self,
        num_obs: int = 50,
        num_actions: int = 10,
        hidden_dims: list[int] | None = None,
    ) -> None:
        super().__init__()
        if hidden_dims is None:
            hidden_dims = [512, 256, 128]

        layers: list[nn.Module] = []
        layers.append(nn.Linear(num_obs, hidden_dims[0]))
        layers.append(nn.ELU())
        for i in range(len(hidden_dims)):
            if i == len(hidden_dims) - 1:
                layers.append(nn.Linear(hidden_dims[i], num_actions))
            else:
                layers.append(nn.Linear(hidden_dims[i], hidden_dims[i + 1]))
                layers.append(nn.ELU())
        self.actor = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.actor(obs)

    @classmethod
    def from_checkpoint(
        cls,
        checkpoint_path: str | Path,
        num_obs: int = 50,
        num_actions: int = 10,
        hidden_dims: list[int] | None = None,
        device: str = "cpu",
    ) -> "ActorMLP":
        """Load actor weights from an rsl_rl ActorCritic checkpoint.

        The checkpoint contains 'model_state_dict' with keys like
        'actor.0.weight', 'actor.0.bias', etc.
        """
        model = cls(num_obs=num_obs, num_actions=num_actions, hidden_dims=hidden_dims)

        checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)
        full_state = checkpoint["model_state_dict"]

        # Extract only actor.* keys and strip the 'actor.' prefix
        actor_state = {
            k.replace("actor.", "", 1): v for k, v in full_state.items() if k.startswith("actor.")
        }
        model.actor.load_state_dict(actor_state)
        model.eval()
        model.to(device)
        return model
