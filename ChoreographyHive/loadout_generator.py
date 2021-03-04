import random
from pathlib import Path

from rlbot.agents.base_loadout_generator import BaseLoadoutGenerator
from rlbot.matchconfig.loadout_config import LoadoutConfig


class LoadoutGenerator(BaseLoadoutGenerator):
    def generate_loadout(self, player_index: int, team: int) -> LoadoutConfig:
        loadout = self.load_cfg_file(Path('bots/default.cfg' if player_index != 60 else 'bots/alpha.cfg'), team)
        return loadout
