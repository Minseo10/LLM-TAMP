import logging
import random
import numpy as np

from envs.constants import COLOR_MAP
from envs.pb_env import PybulletEnv
from utils.io_util import load_json, dump_json
from utils.tamp_util import Action, PrimitiveAction, Parameter
from utils.llm_util import textualize_array
from utils.pybullet_utils import *

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class PackCompactEnv(PybulletEnv):
    """
    An environment for packing boxes in a compact basket.
    Use top grasp.
    """

    def __init__(self):
        super().__init__()

        self._primitive_actions = {
            "pick": PrimitiveAction(name="pick", obj_arity=1),
            "place": PrimitiveAction(
                name="place",
                obj_arity=1,
                parameters={
                    "x": Parameter("x", lower_limit=0.0, upper_limit=1.4),
                    "y": Parameter("y", lower_limit=-1.0, upper_limit=1.0),
                    "theta": Parameter("theta", lower_limit=-3.14, upper_limit=3.14),
                },
            ),
        }

    def __str__(self):
        return "PackCompactEnv"

    def reset(self, json_path, prob_num, prob_idx, trial, basket=None, boxes=None,use_gui=True, domain_name="packing"):
        super().reset(use_gui=use_gui, domain_name=domain_name)  # bring robot

        # create table
        if domain_name == "blocksworld_pr":
            self.create_blocksworld_table()
        elif domain_name == "packing":
            self.create_table()

        # create basket
        if domain_name == "packing":
            self.create_basket(x=basket["x"], y=basket["y"], w=basket["w"], l=basket["l"])
        elif domain_name == "kitchen":
            self.create_box("mystove", BLUE, x=0.5, y=0.0, z=0.025)
            self.create_box("mysink", RED, x=-0.5, y=0.0, z=0.025)

        # create objects
        if domain_name == "packing":
            for box_info in boxes.values():
                self.create_customized_box(
                    name=box_info["name"],
                    color=box_info["color"],
                    w=box_info["w"],
                    l=box_info["l"],
                    h=box_info["h"],
                    x=box_info["x"],
                    y=box_info["y"],
                    z=box_info["z"],
                    theta=np.pi,
                )
        elif domain_name == "blocksworld_pr" or domain_name == "kitchen":
            self.bring_blocks(domain_name, json_path, prob_num, prob_idx, trial)

        # physical simulation
        self.simulate()

        observation = self.get_observation(domain_name=domain_name)
        return observation

    def apply_action(self, action: Action, play_traj: bool = False):
        # sanity check
        if action is None:
            return False, "No action is given!"
        if not action.primitive in self._primitive_actions.values():
            return False, "Unknown primitive action!"
        for obj_name in action.obj_args:
            if not obj_name in self.objects:
                return False, "Unknown object name!"

        if action.traj is not None and len(action.traj) > 0:
            traj = action.traj
        else:
            traj = None

        if action.primitive.name == "pick":
            obj_name = action.obj_args[0]
            object = self.objects[obj_name]

            # prepare obstacles (avoid all other objects)
            obstacles = self.prepare_obstacles(obj_name_list=[obj_name], remove_mode=True)

            success, traj, mp_feedback = self.robot.pick(
                object, obstacles, grasp_direction="top", traj=traj, play_traj=play_traj
            )
            if success:
                logger.debug("Picked!")
            else:
                logger.debug(f"Pick is not executed:{mp_feedback}")

            # don't simulate at pick

        elif action.primitive.name == "place":
            obj_name = action.obj_args[0]
            object = self.objects[obj_name]

            # prepare obstacles (avoid all other objects)
            obstacles = self.prepare_obstacles(obj_name_list=[obj_name], remove_mode=True)

            success, traj, mp_feedback = self.robot.place(
                object,
                obstacles,
                # randomly sample x,y for ablation study
                x=action.param_args["x"],
                y=action.param_args["y"],
                z=0.05,
                theta=action.param_args["theta"],
                traj=traj,
                play_traj=play_traj,
            )
            # only simulate at successful place
            if success:
                self.simulate()
                self.theta_dict[obj_name] = action.param_args["theta"]
                logger.debug("Placed!")
            else:
                logger.debug(f"Place is not executed:{mp_feedback}")

        # assign traj
        if action.traj is None or len(action.traj) == 0:
            action.traj = traj

        return success, mp_feedback

    def get_observation(self, domain_name="packing"):
        observation = super().get_observation()
        # remove table & basket from observation
        if domain_name == "blocksworld_pr" or domain_name == "packing":
            observation.pop("table")
        if domain_name == "packing":
            basket_obs = observation.pop("basket")
            # textualize observation
            # add basket info
            x_range = np.array([basket_obs["bb_min"][0], basket_obs["bb_max"][0]])
            y_range = np.array([basket_obs["bb_min"][1], basket_obs["bb_max"][1]])
            basket_text = f"The basket has a rectangular shape, ranges {textualize_array(x_range)} along the x axis, and ranges {textualize_array(y_range)} along the y axis."
        if domain_name == "kitchen":
            stove_obs = observation.pop("mystove")
            x_range = np.array([stove_obs["bb_min"][0], stove_obs["bb_max"][0]])
            y_range = np.array([stove_obs["bb_min"][1], stove_obs["bb_max"][1]])
            z_range = np.array([stove_obs["bb_min"][2], stove_obs["bb_max"][2]])
            stove_text = f"The stove has a rectangular shape, ranges {textualize_array(x_range)} along the x axis, and ranges {textualize_array(y_range)} along the y axis."

            sink_obs = observation.pop("mysink")
            x_range = np.array([sink_obs["bb_min"][0], sink_obs["bb_max"][0]])
            y_range = np.array([sink_obs["bb_min"][1], sink_obs["bb_max"][1]])
            z_range = np.array([sink_obs["bb_min"][2], sink_obs["bb_max"][2]])
            sink_text = f"The sink has a rectangular shape, ranges {textualize_array(x_range)} along the x axis, and ranges {textualize_array(y_range)} along the y axis."


        # add box info
        boxes_text = f"There are several blocks in the envrionment: {', '.join(observation.keys())}."
        for object_name, object_state in observation.items():
            boxes_text += f"\n{object_name} is at position {textualize_array(object_state['position'])}, and it has min bounding box corner {textualize_array(object_state['bb_min'])} and max bounding box corner {textualize_array(object_state['bb_max'])},"
            width = object_state["bb_max"][0] - object_state["bb_min"][0]
            length = object_state["bb_max"][1] - object_state["bb_min"][1]
            height = object_state["bb_max"][2] - object_state["bb_min"][2]
            boxes_text += f"its length along x axis is {textualize_array(width)}, its length along y axis is {textualize_array(length)}, and it's height along z axis is {textualize_array(height)}."

        # predicate info
        predicate_list = []
        for object_name in observation.keys():
            if self.check_in_basket(object_name):
                predicate_list.append(f"{object_name} is in basket")
            else:
                predicate_list.append(f"{object_name} is not in basket")

        predicate_text = ", ".join(predicate_list) + "."

        if domain_name == "packing":
            obs_text = basket_text + "\n" + boxes_text + "\n" + predicate_text
        elif domain_name == "kitchen":
            obs_text = sink_text + "\n" + stove_text + "\n"+ boxes_text + "\n" + predicate_text
        return observation, obs_text

    def get_symbolic_plan(self):
        return [
            "pick(['red_box'], {})",
            "place(['red_box'], {'x': ?, 'y': ?, 'theta': ?})",
            "pick(['blue_box'], {})",
            "place(['blue_box'], {'x': ?, 'y': ?, 'theta': ?})",
            "pick(['cyan_box'], {})",
            "place(['cyan_box'], {'x': ?, 'y': ?, 'theta': ?})",
            "pick(['yellow_box'], {})",
            "place(['yellow_box'], {'x': ?, 'y': ?, 'theta': ?})",
        ]

    def check_in_basket(self, obj_name, tol=0.015):
        min_bb_basket, max_bb_basket = self.get_bb("basket")
        min_bb_obj, max_bb_obj = self.get_bb(obj_name)

        if (
            min_bb_obj[0] > min_bb_basket[0] - tol
            and max_bb_obj[0] < max_bb_basket[0] + tol
            and min_bb_obj[1] > min_bb_basket[1] - tol
            and max_bb_obj[1] < max_bb_basket[1] + tol
        ):
            return True
        return False

    def check_goal(self):
        is_goal = True
        feedback = []
        for obj_name in self.objects.keys():
            if obj_name != "basket" and obj_name != "table":
                if not self.check_in_basket(obj_name):
                    is_goal = False
                    feedback.append(f"{obj_name} is not in basket")

        return is_goal, ", ".join(feedback)

    def create_task_instances(
        self,
        env_config,
        num_instances,
        save_to_file=False,
        instance_file=None,
        overwrite=False,
    ):
        # if already exists, load from file
        if instance_file is not None and instance_file.exists() and not overwrite:
            task_instances = load_json(instance_file)
            logger.info(f"Load from existing file {instance_file}.")

        else:
            task_instances = {}
            for task_i in range(num_instances):
                basket_info = {
                    "x": env_config.basket.x,
                    "y": env_config.basket.y,
                    "w": env_config.basket.w,
                    "l": env_config.basket.l,
                }

                # sample box locations
                # todo: fill in algorithm for sampling initial box locations
                boxes_info = {}
                xmin, xmax = [0.385, 0.395]
                last_y = -0.6
                gripper_length = 0.09
                for box_i, (box_name, box) in enumerate(env_config.boxes.items()):
                    x = random.uniform(xmin, xmax)
                    y = random.uniform(
                        max(-0.6 + 0.3 * box_i, last_y + box.l / 2),
                        -0.6 + 0.3 * (box_i + 1),
                    )
                    y = max(y, last_y + box.l / 2 + gripper_length)
                    last_y = y + box.l / 2

                    box_info = {
                        "name": box_name,
                        "color": COLOR_MAP[box.color],
                        "w": box.w,
                        "l": box.l,
                        "h": box.h,
                        "x": x,
                        "y": y,
                        "z": 0.06,
                    }

                    boxes_info[box_i] = box_info

                instance = {"basket": basket_info, "boxes": boxes_info}
                task_instances[task_i] = instance

            if save_to_file:
                assert (
                    instance_file is not None
                ), "instance_file must be specified when save_to_file is True"

                dump_json(task_instances, instance_file)

        return task_instances
