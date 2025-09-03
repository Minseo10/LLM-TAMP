import logging
import numpy as np
from pathlib import Path
import pybullet as p
from envs.pack_compact_env import PackCompactEnv
from planners.llm_tamp_planner import LLMTAMPPlanner
from planners.random_param_sampler import RandomParamSampler

from utils.io_util import mkdir, save_npz, dump_json
import time
import multiprocessing as mp

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class TAMPRunner:
    def __init__(self, cfg):
        self.cfg = cfg
        self.env_cfg = cfg.env
        self.planner_cfg = cfg.planner

        # environment
        self.env = PackCompactEnv()
        self.primitive_actions = self.env.primitive_actions

        # save dirs
        self.save_to_file = cfg.save_to_file
        self.world_dir = Path("envs/task_instances")
        mkdir(self.world_dir)
        self.save_dir = Path(cfg.save_dir)
        self.json_path = cfg.json_path
        self.domain_name = cfg.domain_name

        if self.domain_name == "packing":
            env_desc_file = "pack_boxes.txt"
        if self.domain_name == "blocksworld_pr":
            env_desc_file = "blocksworld.txt"
        if self.domain_name == "kitchen":
            env_desc_file = "kitchen.txt"

        # planner
        self.planner = LLMTAMPPlanner(
            planner_prompt_file=self.planner_cfg.planner_prompt_file,
            env_desc_file=env_desc_file,
            primitive_actions=self.primitive_actions,
            with_mp_feedback=self.planner_cfg.with_mp_feedback,
            trace_size=self.planner_cfg.trace_size,
        )
        self.max_llm_calls = cfg.max_llm_calls

        self.play_traj = cfg.play_traj
        self.use_gui = cfg.use_gui

        self.use_hard_timeout = getattr(cfg, "use_hard_timeout", False)
        self.timeout_sec = getattr(cfg, "timeout_sec", 600)

        logger.info(f"Run TAMP for setting {cfg.env.env_name}!")

    # def run_once(self, json_path, prob_num, prob_idx, trial, domain_name):
    #     # main loop
    #     last_feedback_list = []
    #     last_temp_tamp_plan = None
    #     final_tamp_plan = None
    #     num_mp_calls = 0
    #     num_llm_calls = 0
    #     for _ in range(self.max_llm_calls):
    #         # reset environment
    #         obs, obs_text = self.env.reset(json_path=json_path, prob_num=prob_num, prob_idx=prob_idx, trial=trial, use_gui=self.use_gui, domain_name=domain_name)
    #         goal_text, goal_predicate = self.env.get_goal(json_path=json_path, prob_num=prob_num, prob_idx=prob_idx, trial=trial, domain_name=domain_name)
    #         # propose plan with llm (symbolic plan only used when sampling parameters only)
    #         plan = self.planner.plan(
    #             domain_name, prob_num, prob_idx, obs_text, goal_text, last_feedback_list, symbolic_plan=self.env.get_symbolic_plan()
    #         )
    #         last_feedback_list = []  # last feedback
    #         num_llm_calls += 1
    #
    #         # rollout
    #         temp_tamp_plan = []
    #         same_as_last = True
    #         for action_i, action in enumerate(plan):
    #             # if same as last, simulate last traj
    #             if (
    #                 same_as_last
    #                 and last_temp_tamp_plan is not None
    #                 and len(last_temp_tamp_plan) > action_i
    #             ):
    #                 if str(action) == str(last_temp_tamp_plan[action_i]):
    #                     action = last_temp_tamp_plan[action_i]
    #                 else:
    #                     same_as_last = False
    #
    #             # motion planning when no traj
    #             if action.traj is None or len(action.traj) == 0:
    #                 num_mp_calls += 1
    #
    #             _, feedback = self.env.step(json_path, prob_num, prob_idx, trial,
    #                 action, goal_predicate, domain_name=domain_name, play_traj=self.play_traj
    #             )  # this step will also save traj in action
    #             last_feedback_list.append((action, feedback))
    #
    #             logger.debug(f"Apply action: {action}")
    #             logger.debug(f"Succeed: {feedback.action_success}")
    #             logger.debug(f"MP feedback: {feedback.motion_planner_feedback}")
    #
    #             if feedback.action_success:
    #                 temp_tamp_plan.append(action)
    #             else:
    #                 logger.info(f"Action {str(action)} failed!")
    #                 break
    #
    #             if feedback.goal_achieved:
    #                 final_tamp_plan = temp_tamp_plan
    #                 break
    #
    #         last_temp_tamp_plan = temp_tamp_plan
    #         if feedback.goal_achieved:
    #             logger.info("Find full plan!")
    #             break
    #         else:
    #             logger.info(f"Goal not achieved: {feedback.task_process_feedback}")
    #
    #     logger.info("Episode ends!")
    #     self.env.destroy()
    #
    #     episode_data = {
    #         "tamp_plan": final_tamp_plan,
    #         "goal_achieved": feedback.goal_achieved,
    #         "num_mp_calls": num_mp_calls,
    #         "num_llm_calls": num_llm_calls,
    #     }
    #
    #     return episode_data

    def run_once(self, json_path, prob_num, prob_idx, trial, repeat, domain_name):
        # main loop
        last_feedback_list = []
        last_temp_tamp_plan = None
        final_tamp_plan = None
        num_mp_calls = 0
        num_llm_calls = 0

        timeout_sec = self.timeout_sec
        timed_out = False
        error_msg = ""

        save_log_dir = self.save_dir / f"{prob_num}_{prob_idx}_{trial}_{repeat}"
        mkdir(save_log_dir)
        log_file = save_log_dir / f"main.log"
        file_handler = logging.FileHandler(log_file, mode="w", encoding="utf-8")
        file_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter("%(asctime)s [%(levelname)s] - %(message)s")
        file_handler.setFormatter(formatter)

        logger.addHandler(file_handler)
        logger.info(f"=== Start run_once | domain={domain_name}, prob={prob_num}, idx={prob_idx}, trial={trial} ===")

        start_time = time.time()
        end_time = start_time

        try:
            while (time.time() - start_time) < timeout_sec:
                # reset environment
                obs, obs_text = self.env.reset(
                    json_path=json_path,
                    prob_num=prob_num,
                    prob_idx=prob_idx,
                    trial=trial,
                    use_gui=self.use_gui,
                    domain_name=domain_name
                )
                goal_text, goal_predicate = self.env.get_goal(
                    json_path=json_path,
                    prob_num=prob_num,
                    prob_idx=prob_idx,
                    trial=trial,
                    domain_name=domain_name
                )

                # propose plan with llm (symbolic plan only used when sampling parameters only)
                plan = self.planner.plan(
                    domain_name, prob_num, prob_idx, obs_text, goal_text,
                    last_feedback_list, symbolic_plan=self.env.get_symbolic_plan()
                )
                last_feedback_list = []  # last feedback
                num_llm_calls += 1

                # rollout
                temp_tamp_plan = []
                same_as_last = True
                reached_goal = False

                for action_i, action in enumerate(plan):
                    # if same as last, simulate last traj
                    if (
                            same_as_last
                            and last_temp_tamp_plan is not None
                            and len(last_temp_tamp_plan) > action_i
                    ):
                        if str(action) == str(last_temp_tamp_plan[action_i]):
                            action = last_temp_tamp_plan[action_i]
                        else:
                            same_as_last = False

                    # motion planning when no traj
                    if action.traj is None or len(action.traj) == 0:
                        num_mp_calls += 1

                    _, feedback = self.env.step(
                        json_path, prob_num, prob_idx, trial,
                        action, goal_predicate, domain_name=domain_name, play_traj=self.play_traj
                    )  # this step will also save traj in action
                    last_feedback_list.append((action, feedback))

                    logger.debug(f"Apply action: {action}")
                    logger.debug(f"Succeed: {feedback.action_success}")
                    logger.debug(f"MP feedback: {feedback.motion_planner_feedback}")

                    if feedback.action_success:
                        temp_tamp_plan.append(action)
                    else:
                        logger.info(f"Action {str(action)} failed!")
                        break

                    if feedback.goal_achieved:
                        final_tamp_plan = temp_tamp_plan
                        reached_goal = True
                        break

                    # 타임아웃 체크 (긴 플랜에서 너무 오래 굴리지 않도록)
                    if (time.time() - start_time) >= timeout_sec:
                        end_time = time.time()
                        break

                last_temp_tamp_plan = temp_tamp_plan

                if reached_goal:
                    logger.info("Find full plan!")
                    break
                else:
                    logger.info(
                        f"Goal not achieved: {feedback.task_process_feedback if 'feedback' in locals() else 'no feedback'}")

                # 타임아웃 재확인
                if (time.time() - start_time) >= timeout_sec:
                    end_time = time.time()
                    break
        except Exception as e:  # [MOD]
            error_msg = repr(e)
            logger.exception("Unhandled exception in run_once")
        finally:  # [MOD]
            end_time = time.time()
            logger.info("Episode ends!")
            self.env.destroy()
            logger.info(f"=== End run_once | success={final_tamp_plan is not None}, "
                        f"time={end_time - start_time:.2f}s, timed_out={timed_out} ===")
            logger.removeHandler(file_handler)
            file_handler.close()

        goal_achieved_flag = final_tamp_plan is not None

        episode_data = {
            "tamp_plan": final_tamp_plan,
            "goal_achieved": goal_achieved_flag,
            "planning_time": end_time - start_time,
            "num_mp_calls": num_mp_calls,
            "num_llm_calls": num_llm_calls,
            "timed_out": timed_out,
            "error": error_msg,
        }

        return episode_data

    def _run_once_worker(self, q, json_path, prob_num, prob_idx, trial, repeat, domain_name):  # [MOD]
        """
        하드 타임아웃용 워커 프로세스 타겟.
        결과는 Queue로 전달.
        """
        try:
            res = self.run_once(json_path, prob_num, prob_idx, trial, repeat, domain_name)
            q.put(("ok", res))
        except Exception as e:
            q.put(("err", repr(e)))

    def run_once_hard_timeout(self, json_path, prob_num, prob_idx, trial, repeat, domain_name,
                              timeout_sec=None):  # [MOD]
        """
        multiprocessing.Process를 사용한 하드 타임아웃 버전.
        timeout_sec 이 지나면 프로세스를 terminate.
        """
        if timeout_sec is None:
            timeout_sec = self.timeout_sec

        q = mp.Queue()
        p = mp.Process(target=self._run_once_worker,
                       args=(q, json_path, prob_num, prob_idx, trial, repeat, domain_name))
        start = time.time()
        p.start()
        p.join(timeout_sec)

        res = {
                "tamp_plan": None,
                "goal_achieved": False,
                "planning_time": 0,
                "num_mp_calls": 0,
                "num_llm_calls": 0,
                "timed_out": False,  # [MOD]
                "error": "",  # [MOD]
        }

        if p.is_alive():
            # 타임아웃: 프로세스 강제 종료
            print(f"[TIMEOUT] Killing run (>{600}s).")
            p.terminate()
            p.join()
            elapsed = time.time() - start
            # 최소 episode_data 형태로 반환
            return res.update({
                "tamp_plan": None,
                "goal_achieved": False,
                "planning_time": elapsed,
                "num_mp_calls": 0,
                "num_llm_calls": 0,
                "timed_out": True,  # [MOD]
                "error": "",  # [MOD]
            })
        else:
            try:
                status, payload = q.get_nowait()
                if status == "ok":
                    res.update(payload)
                else:
                    print("[ERROR] run failed in child process.")
                    print(payload)
                    res.update({
                        "planning_time": time.time() - start,
                        "error": payload,
                    })
            except Exception as e:
                # child crashed without posting a result
                res.update({
                    "planning_time": time.time() - start,
                    "error": f"no result from child: {e}",
                })

    def _append_csv_row(self, csv_path: Path, fieldnames, row: dict):
        import csv
        import os

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        file_exists = csv_path.exists()
        with open(csv_path, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                w.writeheader()
            w.writerow({k: row.get(k, "") for k in fieldnames})
            f.flush()
            os.fsync(f.fileno())

    def run(self, prob_num_range=[3, 4, 5, 6], prob_idx_range=[1, 2, 3, 4, 5], trial_range=[1, 2,], repeat_range=[1, 2, 3]):
        # task_instances = self.env.create_task_instances(
        #     self.env_cfg,
        #     self.env_cfg.task_instances,
        #     save_to_file=self.save_to_file,
        #     instance_file=self.world_dir / f"{self.env_cfg.env_name}.json",
        #     overwrite=self.cfg.overwrite_instances,
        # )

        goal_achieved_list = []
        num_steps_list = []
        num_mp_calls_list = []
        num_llm_calls_list = []
        results_csv = Path(f"/home/minseo/develop/LLM-TAMP/experiments/{self.domain_name}/summary.csv")
        Path(results_csv).parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "prob_num", "prob_idx", "trial", "repeat",
            "total_planning_time",
            "success", "num_llm_calls", "num_mp_calls", "timed_out", "sim_success", "error"
        ]

        # for idx, task_config in task_instances.items():
        for prob_num in prob_num_range:
            for prob_idx in prob_idx_range:
                for trial in trial_range:
                    for repeat in repeat_range:
                        # reset planner
                        self.planner.reset()
                        if self.use_hard_timeout:
                            episode_data = self.run_once_hard_timeout(
                                self.json_path, prob_num, prob_idx, trial, repeat, domain_name=self.domain_name,
                                timeout_sec=self.timeout_sec
                            )
                        else:
                            episode_data = self.run_once(
                                self.json_path, prob_num, prob_idx, trial, repeat, domain_name=self.domain_name
                            )

                        goal_achieved = episode_data["goal_achieved"]
                        num_llm_calls = episode_data["num_llm_calls"]
                        num_mp_calls = episode_data["num_mp_calls"]
                        planning_time = episode_data["planning_time"]
                        timed_out = episode_data.get("timed_out", False)
                        error_msg = episode_data.get("error", "")

                        logger.info(f"Goal achieved: {goal_achieved}")
                        logger.info(f"Number of MP calls: {num_mp_calls}")
                        logger.info(f"Number of LLM calls: {num_llm_calls}")
                        if timed_out:
                            logger.info(f"Episode timed out at {planning_time:.2f}s")
                        if error_msg:
                            logger.info(f"Episode error: {error_msg}")

                        goal_achieved_list.append(goal_achieved)

                        if goal_achieved:
                            num_steps_list.append(len(episode_data["tamp_plan"]))
                        else:
                            num_steps_list.append(-1)
                        num_mp_calls_list.append(episode_data["num_mp_calls"])
                        num_llm_calls_list.append(episode_data["num_llm_calls"])

                        if self.save_to_file:
                            # save tamp_plan into npz
                            save_episode_dir = self.save_dir / f"{prob_num}_{prob_idx}_{trial}_{repeat}"
                            mkdir(save_episode_dir)
                            # import pdb

                            # pdb.set_trace()
                            save_npz(episode_data, save_episode_dir / "result.npz")

                            # save json every time
                            json_data = {
                                "success_rate": np.mean(goal_achieved_list),
                                "goal_achieved": goal_achieved_list,
                                "num_steps": num_steps_list,
                                "num_mp_calls": num_mp_calls_list,
                                "num_llm_calls": num_llm_calls_list,
                            }
                            dump_json(json_data, self.save_dir / "result.json")

                            # save in csv
                            res = {
                                "prob_num": prob_num,
                                "prob_idx": prob_idx,
                                "trial": trial,
                                "repeat": repeat,
                                "total_planning_time": planning_time,
                                "success": bool(goal_achieved),
                                "num_llm_calls": num_llm_calls,
                                "num_mp_calls": num_mp_calls,  # [MOD] 누락 보완
                                "timed_out": bool(timed_out),
                                "sim_success": None,
                                "error": error_msg,
                            }

                            self._append_csv_row(results_csv, fieldnames, res)



class RandomSampleRunner(TAMPRunner):
    def __init__(self, cfg):
        self.cfg = cfg
        self.env_cfg = cfg.env

        # environment
        self.env = PackCompactEnv()
        self.primitive_actions = self.env.primitive_actions

        # save dirs
        self.save_to_file = cfg.save_to_file
        self.world_dir = Path("envs/task_instances")
        mkdir(self.world_dir)
        self.save_dir = Path(cfg.save_dir)

        # planner
        self.planner = RandomParamSampler(primitive_actions=self.primitive_actions)
        self.max_sample_iters = cfg.max_sample_iters

        self.play_traj = cfg.play_traj
        self.use_gui = cfg.use_gui

        logger.info(f"Run parameter sampling for setting {cfg.env.env_name}!")

    def run_once(self, task_config):
        # main loop
        last_temp_tamp_plan = None
        final_tamp_plan = None
        num_mp_calls = 0
        num_sample_iters = 0
        while True:
            # reset environment
            obs, obs_text = self.env.reset(**task_config, use_gui=self.use_gui)
            bb_min, bb_max = self.env.get_bb("basket")
            x_range = [bb_min[0], bb_max[0]]
            y_range = [bb_min[1], bb_max[1]]

            # x_range = [0, 1]
            # y_range = [-1, 1]
            theta_range = [-np.pi, np.pi]

            # propose plan with llm (symbolic plan only used when sampling parameters only)
            plan = self.planner.plan(x_range, y_range, theta_range)
            num_sample_iters += 1

            # rollout
            temp_tamp_plan = []
            same_as_last = True
            for action_i, action in enumerate(plan):
                # if same as last, simulate last traj
                if (
                    same_as_last
                    and last_temp_tamp_plan is not None
                    and len(last_temp_tamp_plan) > action_i
                ):
                    if str(action) == str(last_temp_tamp_plan[action_i]):
                        action = last_temp_tamp_plan[action_i]
                    else:
                        same_as_last = False

                # motion planning when no traj
                if action.traj is None or len(action.traj) == 0:
                    num_mp_calls += 1

                _, feedback = self.env.step(
                    action, play_traj=self.play_traj
                )  # this step will also save traj in action

                logger.debug(f"Apply action: {action}")
                logger.debug(f"Succeed: {feedback.action_success}")

                if feedback.action_success:
                    temp_tamp_plan.append(action)
                else:
                    logger.info(f"Action {str(action)} failed!")
                    break

                if feedback.goal_achieved:
                    final_tamp_plan = temp_tamp_plan
                    break

            last_temp_tamp_plan = temp_tamp_plan
            if feedback.goal_achieved:
                logger.info("Find full plan!")
                break
            else:
                logger.info(f"Goal not achieved: {feedback.task_process_feedback}")

            if num_sample_iters >= self.max_sample_iters:
                logger.info("Reach max sample iters!")
                break

        logger.info("Episode ends!")
        self.env.destroy()

        episode_data = {
            "tamp_plan": final_tamp_plan,
            "goal_achieved": feedback.goal_achieved,
            "num_mp_calls": num_mp_calls,
            "num_sample_iters": num_sample_iters,
        }

        return episode_data

    def run(self):
        task_instances = self.env.create_task_instances(
            self.env_cfg,
            self.env_cfg.task_instances,
            save_to_file=self.save_to_file,
            instance_file=self.world_dir / f"{self.env_cfg.env_name}.json",
            overwrite=self.cfg.overwrite_instances,
        )

        goal_achieved_list = []
        num_steps_list = []
        num_mp_calls_list = []
        num_sample_iters_list = []
        for idx, task_config in task_instances.items():
            episode_data = self.run_once(task_config)

            goal_achieved = episode_data["goal_achieved"]
            num_sample_iters = episode_data["num_sample_iters"]
            num_mp_calls = episode_data["num_mp_calls"]

            logger.info(f"Goal achieved: {goal_achieved}")
            logger.info(f"Number of MP calls: {num_mp_calls}")
            logger.info(f"Number of sample iters: {num_sample_iters}")

            goal_achieved_list.append(goal_achieved)

            if goal_achieved:
                num_steps_list.append(len(episode_data["tamp_plan"]))
            else:
                num_steps_list.append(-1)
            num_mp_calls_list.append(episode_data["num_mp_calls"])
            num_sample_iters_list.append(episode_data["num_sample_iters"])

            if self.save_to_file:
                # save tamp_plan into npz
                save_episode_dir = self.save_dir / f"{idx}"
                mkdir(save_episode_dir)
                save_npz(episode_data, save_episode_dir / "result.npz")

        if self.save_to_file:
            json_data = {
                "success_rate": np.mean(goal_achieved_list),
                "goal_achieved": goal_achieved_list,
                "num_steps": num_steps_list,
                "num_mp_calls": num_mp_calls_list,
                "num_sample_iters": num_sample_iters_list,
            }
            dump_json(json_data, self.save_dir / "result.json")
