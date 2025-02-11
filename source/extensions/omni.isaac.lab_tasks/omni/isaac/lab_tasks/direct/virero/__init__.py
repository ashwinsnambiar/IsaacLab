import gymnasium as gym

from . import agents_toedit

# Register Gym environments.

gym.register(
    id="Isaac-Virero-Sia20f-Control-v0",
    entry_point=f"{__name__}.virero_sia20f_env:VireroSia20fEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.virero_sia20f_env:VireroSia20fEnvCfg",
        "rl_games_cfg_entry_point": f"{agents_toedit.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents_toedit.__name__}.rsl_rl_ppo_cfg:FrankaCabinetPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents_toedit.__name__}:skrl_ppo_cfg.yaml",
    },
)

