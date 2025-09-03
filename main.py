import hydra
import logging
from pathlib import Path
from omegaconf import DictConfig, OmegaConf

from runners import TAMPRunner, RandomSampleRunner
from utils.log_util import setup_global_logger

RUNNER = {
    "tamp": TAMPRunner,
    "random_sample": RandomSampleRunner,
}


@hydra.main(version_base=None, config_path="configs")
def main(cfg: DictConfig):
    assert len(cfg) > 0, "No config file specified!"
    save_dir = Path(cfg.save_dir)

    # dump config
    OmegaConf.save(cfg, save_dir / "config.yaml")

    # setup logger
    logger = logging.getLogger()
    setup_global_logger(logger, file=save_dir / "main.log")

    # runner
    assert cfg.runner in RUNNER, f"Runner {cfg.runner} not implemented!"
    runner_cls = RUNNER[cfg.runner]
    runner = runner_cls(cfg)
    runner.run(prob_num_range=cfg.prob_num, prob_idx_range=cfg.prob_idx, trial_range=cfg.trial, repeat_range=cfg.repeat)
    # runner.run(prob_num_range=[4,], prob_idx_range=[1,], trial_range=[1,])


if __name__ == "__main__":
    main()
