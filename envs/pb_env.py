import os
import abc
import logging
import numpy as np
from scipy.spatial.transform import Rotation

import pybullet as p
from pybullet_planning import (
    connect,
    disconnect,
    is_connected,
    create_obj,
    set_pose,
    Pose,
    load_pybullet,
    get_movable_joints,
    link_from_name,
    set_joint_positions,
    get_joint_positions,
    create_attachment,
    get_link_pose,
    get_distance,
    quat_angle_between,
    wait_for_duration,
    multiply,
)

from utils.planning_util import plan_joint_motion, check_ee_collision
from utils.pb_util import (
    assign_link_colors,
    create_box,
    set_point,
    get_pose,
    set_static,
    add_data_path
)
from utils.tamp_util import Action, TAMPFeedback, find_entry
from envs.constants import ASSETS_DIR, COLOR_FRANKA, FRANKA_Limits, BROWN, TAN, PR2_Limits, KUKA_Limits
from utils.pybullet_utils import *
from utils.pr2_utils import *
logger = logging.getLogger(__name__)


BOX_PATH = os.path.join(ASSETS_DIR, "box_obstacle3.obj")
HOOK_PATH = os.path.join(ASSETS_DIR, "hook.obj")
FRANKA_ROBOT_URDF = os.path.join(ASSETS_DIR, "franka_description", "robots", "panda_arm_hand.urdf")
PR2_URDF = os.path.join(ASSETS_DIR, "pr2_description", "pr2_modified.urdf")
KUKA_URDF = os.path.join(ASSETS_DIR,"drake", "iiwa_description", "urdf", "iiwa14_polytope_collision.urdf")

class PybulletEnv:
    def __init__(self):
        self._objects = {}
        self._primitive_actions = {}
        self.theta_dict = {}

    @property
    def objects(self):
        return self._objects

    @property
    def primitive_actions(self):
        return self._primitive_actions

    @property
    def body_id_name_mapping(self):
        return {v: k for k, v in self._objects.items()}

    def reset(self, use_gui=True, domain_name="packing"):
        # destroy the current simulation
        self.destroy()

        # connect to pybullet
        connect(use_gui=use_gui)
        p.setGravity(0, 0, -9.8)

        # add floor
        add_data_path()
        p.loadURDF("plane.urdf")

        # reset objects
        self._objects = {}

        # add pybullet robot
        self.robot = PyBulletRobot(domain_name=domain_name)

        # reset camera
        p.resetDebugVisualizerCamera(
            cameraDistance=2.6,
            cameraYaw=40,
            cameraPitch=-45,
            cameraTargetPosition=[-0.3, 0.5, -1],
        )

    def destroy(self):
        if is_connected():
            disconnect()

    def simulate(self, num_steps=100):
        for _ in range(num_steps):
            p.stepSimulation()

    def collision_function(self, xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2):
        if xmin1 < xmax2 and xmax1 > xmin2 and ymin1 < ymax2 and ymax1 > ymin2:
            return True
        return False

    def step(self,json_path, prob_num, prob_idx, trial, action: Action, goal_list, domain_name:str, *args, **kwargs):
        # apply action
        success, mp_feedback = self.apply_action(domain_name, action, *args, **kwargs) # TODO 여기서 TAMP Plan 같이 리턴

        # get observation
        observation = self.get_observation(json_path, prob_num, prob_idx, trial, domain_name)

        # check goal
        goal_achieved, goal_feedback = self.check_goal(domain_name, goal_list)

        # prepare feedback
        feedback = TAMPFeedback(
            motion_planner_feedback=mp_feedback,
            task_process_feedback=goal_feedback,
            action_success=success,
            goal_achieved=goal_achieved,
        )
        logger.info(feedback.motion_planner_feedback)

        if goal_achieved:
            logger.debug("Goal achieved!")

        return observation, feedback

    def get_observation(self, json_path, prob_num, prob_idx, trial, domain_name):
        # in this general function, we assume getting observations for every objects
        # cprint("here is in the observation function", "yellow")
        observation = {}
        for obj_name, obj in self.objects.items():
            # for position
            pos = self.get_position(obj_name)
            # bbox = self.get_bounding_box(obj_name)
            bb_min, bb_max = self.get_bb(obj_name)

            observation[obj_name] = {"position": pos, "bb_min": bb_min, "bb_max": bb_max}
        return observation

    @abc.abstractmethod
    def apply_action(self):
        raise NotImplementedError("Override me!")

    @abc.abstractmethod
    def check_goal(self):
        raise NotImplementedError("Override me!")

    @abc.abstractmethod
    def create_task_instances(self):
        raise NotImplementedError("Override me!")

    def create_table(self, name="table", color=TAN, x=0.1, y=0, z=0):  # -0.005):
        table_body = create_box(w=3, l=3, h=0.01, color=color)
        set_point(table_body, [x, y, z])
        self._objects[name] = table_body

    def create_blocksworld_table(self, name="table", width=0.6, length=1.4, height=0.71, thickness=0.03, radius=0.015,
                     top_color=(0.7, 0.7, 0.7, 1.), leg_color=TAN, cylinder=True, center=(0.55, 0.0), **kwargs):
        # TODO: table URDF
        surface = get_box_geometry(width, length, thickness)
        surface_pose = Pose(Point(x=center[0], y=center[1], z=height - thickness / 2.))  # x=0.55, y=0.0,

        leg_height = height - thickness
        if cylinder:
            leg_geometry = get_cylinder_geometry(radius, leg_height)
        else:
            leg_geometry = get_box_geometry(width=2 * radius, length=2 * radius, height=leg_height)
        legs = [leg_geometry for _ in range(4)]
        leg_center = np.array([width, length]) / 2. - radius * np.ones(2)
        leg_xys_local = [np.multiply(leg_center, np.array(signs))
                         for signs in product([-1, +1], repeat=len(leg_center))]
        leg_poses = [Pose(point=[center[0] + x, center[1] + y, leg_height / 2.])
                     for x, y in leg_xys_local]

        geoms = [surface] + legs
        poses = [surface_pose] + leg_poses
        colors = [top_color] + len(legs) * [leg_color]

        collision_id, visual_id = create_shape_array(geoms, poses, colors)
        body = create_body(collision_id, visual_id, **kwargs)

        # TODO: unable to use several colors
        # for idx, color in enumerate(geoms):
        #    set_color(body, shape_index=idx, color=color)
        self._objects[name] = body
        return body

    def create_basket(self, name="basket", color=BROWN, x=0.6, y=0, z=0.002, w=0.2, l=0.4, h=0.01):
        basket_body = create_box(w=w, l=l, h=h, color=color)
        set_point(basket_body, [x, y, z])
        self._objects[name] = basket_body

    def create_customized_box(self, name, color, w, l, h, x, y, z, theta=np.pi):
        body = create_box(w=w, l=l, h=h, color=color, mass=1.0)
        set_pose(body, Pose(point=[x, y, z], euler=np.array([theta, 0, 0])))
        self._objects[name] = body
        self.theta_dict[name] = 0

    def create_box(self, name, color, x, y, z, theta=np.pi):
        box_body = create_obj(BOX_PATH, scale=0.5, color=color, mass=1.0)
        set_pose(box_body, Pose(point=[x, y, z], euler=np.array([theta, 0, 0])))
        self._objects[name] = box_body
        self.theta_dict[name] = 0

    def bring_blocks(self, domain_name, json_path, prob_num, prob_idx, trial):
        with open(json_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        entry = find_entry(meta, prob_num, prob_idx, trial)

        if not entry or "objects" not in entry:
            print(f"[bring_blocks] No entry/objects for (num={prob_num}, idx={prob_idx}, trial={trial})")
            return

        for name, blk in entry["objects"].items():
            size = blk.get("size", [0.08, 0.08, 0.08])
            pos = blk["pose"]["position"]  # [x, y, z]
            quat = blk["pose"]["quaternion"]  # [x, y, z, w]

            if name not in self._objects:
                self.create_customized_box(
                    name=name,
                    color=COLOR_MAP[name] if domain_name=='blocksworld_pr' else food_specs[name],
                    w=size[0], l=size[1], h=size[2],
                    x=pos[0], y=pos[1], z=pos[2],
                    theta=0.0
                )

            bid = self._objects[name]
            set_pose(bid, (tuple(pos), tuple(quat)))
            from scipy.spatial.transform import Rotation as R
            rot = R.from_quat(quat)
            roll, pitch, yaw = rot.as_euler('xyz', degrees=False)
            self.theta_dict[name] = float(yaw)


    def create_tool(self, name, color, x, y, z, theta=np.pi):
        tool_body = create_obj(HOOK_PATH, scale=0.3, color=color, mass=1.0)
        set_pose(tool_body, Pose(point=[x, y, z], euler=np.array([theta, 0, 0])))
        self._objects[name] = tool_body

    def get_position(self, name):
        return get_pose(self._objects[name])[0]

    def get_orientation(self, name):
        return get_pose(self._objects[name])[1]

    def get_bb(self, name):
        return p.getAABB(self._objects[name])

    def prepare_obstacles(self, obj_name_list=[], remove_mode=False):
        obstacles = {}
        # if remove mode, remove objects in obj_name_list
        if remove_mode:
            for obj_name, id in self._objects.items():
                if obj_name not in obj_name_list:
                    obstacles[id] = obj_name
        else:
            # add obstacles
            for obj_name in obj_name_list:
                obstacles[self._objects[obj_name]] = obj_name

        return obstacles


class PyBulletRobot:
    def __init__(self, domain_name):
        # load robot: Franka FR3 for now
        if domain_name == "packing":
            self.robot = load_pybullet(FRANKA_ROBOT_URDF, fixed_base=True)
            tool_link = "tool_link"
            self.ik_joints = get_movable_joints(self.robot)
            self.joint_limits = FRANKA_Limits
        elif domain_name == "blocksworld_pr":
            self.robot = load_pybullet(PR2_URDF, fixed_base=True)
            tool_link = "l_gripper_tool_frame"
            # self.ik_joints = get_movable_joints(self.robot)
            self.ik_joints = joints_from_names(self.robot, PR2_GROUPS['left_arm'])
            print("moveable joints: ", get_movable_joints(self.robot))
            self.joint_limits = get_all_joint_limits(self.robot, self.ik_joints)
            # self.joint_limits = PR2_Limits

        elif domain_name == "kitchen":
            self.robot = load_pybullet(KUKA_URDF, fixed_base=True)
            tool_link = 'iiwa_link_ee_kuka'
            self.ik_joints = get_movable_joints(self.robot)
            # self.joint_limits = get_all_joint_limits(self.robot, self.ik_joints)
            self.joint_limits = KUKA_Limits

        print("joint limits:", self.joint_limits)

        # assign_link_colors(self.robot, colors=COLOR_FRANKA)
        self.tool_attach_link = link_from_name(self.robot, tool_link)

        # set static: important that the robot is static during planning
        set_static(self.robot)

        self.tcp_offset_tool = np.array([0.0, 0.0, 0.23])

        # grasping attachments & direction
        self.attachments_robot = []
        self.last_grasp_direction = None

        # set grasping offset: position and orientation
        if domain_name == "packing":
            self.position_offset_dict = {
                "top": np.array([0.0, 0, 0.05]),
                "left": np.array([0.5, -0.15, 0]),
                "right": np.array([0, 0.08, 0]),
                "forward": np.array([-0.1, 0, 0]),
            }

            x_axis_positive = [0, 0.7071068, 0, 0.7071068]
            x_axis_negative = [0, -0.7071068, 0, 0.7071068]
            y_axis_negative = [0.7068252, 0, 0, 0.7073883]
            y_axis_positive = [-0.7068252, 0, 0, 0.7073883]
            z_axis_positive = [0, 0, 0, 1]
            z_axis_negative = [1, 0, 0, 0]
            self.rotation_offset_dict = {
                "top": z_axis_negative,
                "left": y_axis_positive,
                "right": y_axis_negative,
                "forward": x_axis_positive,
                "backward": x_axis_negative,
            }
        elif domain_name == "blocksworld_pr":
            self.position_offset_dict = {
                "top": np.array([0.0, 0, 0.04]),
                "left": np.array([0.5, -0.15, 0]),
                "right": np.array([0, 0.08, 0]),
                "forward": np.array([-0.1, 0, 0]),
            }

            x_axis_positive = [0, 0.7071068, 0, 0.7071068]
            x_axis_negative = [0, -0.7071068, 0, 0.7071068]
            y_axis_negative = [0.7068252, 0, 0, 0.7073883]
            y_axis_positive = [-0.7068252, 0, 0, 0.7073883]
            z_axis_positive = [0, 0, 0, 1]
            z_axis_negative = [1, 0, 0, 0]
            # [-0.7068252, 0.0, -0.7068252, 0.0]
            # 0.4999998, 0.4999998, 0.4996018, 0.5003982
            # -0.4999998, 0.4999998, -0.4996018, 0.5003982

            self.rotation_offset_dict = {  # TODO
                "top": x_axis_positive,
                "left": y_axis_positive,
                "right": y_axis_negative,
                "forward": x_axis_positive,
                "backward": x_axis_negative,
            }
            self.pregrasp_offset = 0.10
            self.lift_offset = 0.15
        elif domain_name == "kitchen":
            self.position_offset_dict = {
                "top": np.array([0.0, 0, 0.05]),
                "left": np.array([0.5, -0.15, 0]),
                "right": np.array([0, 0.08, 0]),
                "forward": np.array([-0.1, 0, 0]),
            }

            x_axis_positive = [0, 0.7071068, 0, 0.7071068]
            x_axis_negative = [0, -0.7071068, 0, 0.7071068]
            y_axis_negative = [0.7068252, 0, 0, 0.7073883]
            y_axis_positive = [-0.7068252, 0, 0, 0.7073883]
            z_axis_positive = [0, 0, 0, 1]
            z_axis_negative = [1, 0, 0, 0]
            self.rotation_offset_dict = {
                "top": z_axis_negative,  # TODO
                "left": y_axis_positive,
                "right": y_axis_negative,
                "forward": x_axis_positive,
                "backward": x_axis_negative,
            }
            self.pregrasp_offset = 0.10
            self.lift_offset = 0.10

        # initialize pose
        self.initialize_pose(domain_name)

        ee_pose = get_link_pose(self.robot, self.tool_attach_link)
        print("tool attach link:", self.tool_attach_link)
        print("current ee pose:", ee_pose)

    def initialize_pose(self,domain_name="packing"):
        if domain_name == "packing":
            home_conf = [0, -0.785398163397, 0, -2.35619449, 0, 1.57079632679, 0.78539816, 0.04, 0.04]
            set_joint_positions(self.robot, self.ik_joints, home_conf)
        elif domain_name == "blocksworld_pr":
            set_joint_positions(self.robot, joints_from_names(self.robot, PR2_GROUPS['base']), [0.0, 0.0 ,0.0])
            set_joint_positions(self.robot, joints_from_names(self.robot, PR2_GROUPS['left_arm']), TOP_HOLDING_LEFT_ARM)
            set_joint_positions(self.robot, joints_from_names(self.robot, PR2_GROUPS['left_gripper']), [0.54, 0.54, 0.54, 0.54])

            set_joint_positions(self.robot, joints_from_names(self.robot, PR2_GROUPS['right_arm']), REST_RIGHT_ARM)
            set_joint_positions(self.robot, joints_from_names(self.robot, PR2_GROUPS['torso']), [0.24])
        elif domain_name == "kitchen":
            home_conf = [0, 0, 0, 0, 0, 0, 0]
            set_joint_positions(self.robot, self.ik_joints, home_conf)

    def _tool_target_from_tcp(self, p_tcp_world, quat_tool_world):
        R_tool = Rotation.from_quat(quat_tool_world).as_matrix()
        p_tool = np.asarray(p_tcp_world) - R_tool @ self.tcp_offset_tool
        return p_tool, quat_tool_world

    # def pick(self, domain_name, object, obstacles, grasp_direction, traj=None, play_traj=True):
    #     assert grasp_direction in self.position_offset_dict.keys(), "Unknown grasp direction!"
    #     if len(self.attachments_robot) > 0:
    #         self.release_gripper()
    #     # prepare grasping pose
    #     current_conf = get_joint_positions(self.robot, self.ik_joints)
    #     box_position, box_orientation = get_pose(object)
    #     ee_grasp_position = box_position + self.position_offset_dict[grasp_direction]
    #     ee_grasp_orientation = self.rotation_offset_dict[grasp_direction]
    #
    #     # pre grasp
    #     ee_pre_pos = (np.asarray(ee_grasp_position) + self.pregrasp_offset * np.asarray([0, 0, 1])).tolist()
    #     ee_pre_ori = ee_grasp_orientation
    #
    #     # lift
    #     ee_lift_pos = (np.asarray(ee_grasp_position) + self.lift_offset * np.asarray([0, 0, 1])).tolist()
    #     ee_lift_ori = ee_grasp_orientation
    #
    #     print("Picking")
    #     print(f"position of {object}: {box_position}, orientation of {object}: {box_orientation}")
    #     print(f"ee_grasp_position of {ee_grasp_position}", f"ee_grasp_orientation of {ee_grasp_orientation}")
    #     print("obstacles:", obstacles)
    #     print("attachments_robot:", self.attachments_robot)
    #     print("current conf:", current_conf)
    #
    #     if traj is None:
    #         success, traj, feedback = self.motion_planning(
    #             domain_name,
    #             self.robot,
    #             self.ik_joints,
    #             ee_grasp_position,
    #             ee_grasp_orientation,
    #             obstacles,
    #             self.attachments_robot,
    #             self.joint_limits,
    #         )
    #     else:
    #         assert (
    #             traj[0] == current_conf
    #         ), f"The start conf of the known trajectory {traj[0]} is not the same as the robot start conf {current_conf}"
    #
    #         success = True
    #         feedback = "Success"
    #
    #     # update attachments
    #     if success:
    #         # simulate action
    #         ee_link_from_tcp = Pose(point=(0, 0.00, 0.0))
    #         self.simulate_traj(
    #             self.robot,
    #             self.ik_joints,
    #             self.attachments_robot,
    #             None,
    #             self.tool_attach_link,
    #             ee_link_from_tcp,
    #             traj,
    #             play_traj,
    #         )
    #
    #         lifted_position = [box_position[0], box_position[1], box_position[2] + 0.01]
    #         set_point(object, lifted_position)
    #         box_attach = create_attachment(self.robot, self.tool_attach_link, object)
    #         box_attach.assign()
    #         self.attachments_robot.append(box_attach)
    #         logger.debug(f"box_position: {box_position}")
    #
    #         # set last grasp direction
    #         self.last_grasp_direction = grasp_direction
    #
    #     return success, traj, feedback

    def pick(self, domain_name, object, obstacles, theta, grasp_direction, traj=None, play_traj=True):
        assert grasp_direction in self.position_offset_dict.keys(), "Unknown grasp direction!"
        if len(self.attachments_robot) > 0:
            self.release_gripper()
        # prepare grasping pose
        current_conf = get_joint_positions(self.robot, self.ik_joints)
        box_position, box_orientation = get_pose(object)
        ee_grasp_position = box_position + self.position_offset_dict[grasp_direction]
        # ee_grasp_orientation = self.rotation_offset_dict[grasp_direction]

        if domain_name == "kitchen":
            rot = Rotation.from_euler("XYZ", [np.pi, 0, theta], degrees=False)
            default_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif domain_name == "blocksworld_pr":
            rot = Rotation.from_euler("XYZ", [theta, np.pi/2, 0], degrees=False)
            default_conf = TOP_HOLDING_LEFT_ARM
        quat = rot.as_quat()
        ee_grasp_orientation = quat

        # pre grasp
        ee_pre_pos = (np.asarray(ee_grasp_position) + self.pregrasp_offset * np.asarray([0, 0, 1])).tolist()
        ee_pre_ori = ee_grasp_orientation

        # lift
        ee_lift_pos = (np.asarray(ee_grasp_position) + self.lift_offset * np.asarray([0, 0, 1])).tolist()
        ee_lift_ori = ee_grasp_orientation

        print("Picking")
        print(f"position of {object}: {box_position}, orientation of {object}: {box_orientation}")
        print("obstacles:", obstacles)
        print("attachments_robot:", self.attachments_robot)
        print("current conf:", current_conf)

        if traj is None:
            # ---- 1) PRE-GRASP: current -> pre-grasp ----
            print("MOVING TO PRE GRASP")
            print(f"ee_pre_pos of {ee_pre_pos}", f"ee_pre_ori of {ee_pre_ori}")

            ok1, path1, fb1 = self.motion_planning(
                domain_name,
                self.robot,
                self.ik_joints,
                ee_pre_pos,
                ee_pre_ori,
                obstacles,
                self.attachments_robot,
                self.joint_limits,
            )
            if not ok1:
                return False, None, f"Failed to reach pre-grasp: {fb1}"

            ee_link_from_tcp = Pose(point=(0, 0.00, 0.0))
            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                path1,
                play_traj,
            )

            # ---- 2) GRASP: pre-grasp -> grasp ----
            print("MOVE CLOSER")
            print(f"ee_grasp_position of {ee_grasp_position}", f"ee_grasp_orientation of {ee_grasp_orientation}")

            ok2, path2, fb2 = self.motion_planning(
                domain_name,
                self.robot,
                self.ik_joints,
                ee_grasp_position,
                ee_grasp_orientation,
                obstacles,
                self.attachments_robot,
                self.joint_limits,
            )
            if not ok2:
                set_joint_positions(self.robot, self.ik_joints, current_conf)
                return False, None, f"Failed to reach grasp: {fb2}"

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                path2,
                play_traj,
            )

            # ---- 3) ATTACH ----
            print("ATTACH OBJECT")
            lifted_position = [box_position[0], box_position[1], box_position[2] + 0.01]
            set_point(object, lifted_position)
            box_attach = create_attachment(self.robot, self.tool_attach_link, object)
            box_attach.assign()
            self.attachments_robot.append(box_attach)
            logger.debug(f"box_position: {box_position}")

            for _ in range(3):
                p.stepSimulation()

            # set last grasp direction
            self.last_grasp_direction = grasp_direction

            # set_joint_positions(self.robot, self.ik_joints, path2[-1])

            # ---- 4) LIFT: grasp -> lift ----
            print("LIFT")
            print(f"ee_lift_pos of {ee_lift_pos}", f"ee_lift_ori of {ee_lift_ori}")
            only_stove_sink = {k: v for k, v in obstacles.items() if v in ["stove", "sink"]}
            print("obstacles:", only_stove_sink)
            ok3, path3, fb3 = self.motion_planning(
                domain_name,
                self.robot,
                self.ik_joints,
                ee_lift_pos,
                ee_lift_ori,
                obstacles,
                self.attachments_robot,
                self.joint_limits,
            )
            if not ok3:
                self.release_gripper()
                set_joint_positions(self.robot, self.ik_joints, current_conf)
                return False, None, f"Failed to lift after grasp: {fb3}"

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                path3,
                play_traj,
            )

            print("MOVE THE DEFAULT POSE")
            path4, fb4 = plan_joint_motion(
                self.robot,
                self.ik_joints,
                default_conf,
                obstacles=list(obstacles.keys()),
                attachments=self.attachments_robot,
                self_collisions=False,
                custom_limits=self.joint_limits,
                diagnosis=False,
            )
            if path4 is None:
                logger.debug(fb4)
                set_joint_positions(self.robot, self.ik_joints, current_conf)
                return False, None, f"Failed to move to rest pose after grasp: {fb4}"

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                path4,
                play_traj,
            )

            traj = [path1, path2, path3, path4]
            success = True
            feedback = "Success"

        else:
            assert (
                    traj[0][0] == current_conf
            ), f"The start conf of the known trajectory {traj[0][0]} is not the same as the robot start conf {current_conf}"
            success = True
            feedback = "Success"

            ee_link_from_tcp = Pose(point=(0, 0.00, 0.0))
            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[0],
                play_traj,
            )

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[1],
                play_traj,
            )

            lifted_position = [box_position[0], box_position[1], box_position[2] + 0.01]
            set_point(object, lifted_position)
            box_attach = create_attachment(self.robot, self.tool_attach_link, object)
            box_attach.assign()
            self.attachments_robot.append(box_attach)
            logger.debug(f"box_position: {box_position}")
            self.last_grasp_direction = grasp_direction

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[2],
                play_traj,
            )

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[3],
                play_traj,
            )
        return success, traj, feedback

    # def place(self, domain_name, object, obstacles, x, y, z, theta, traj=None, play_traj=True):
    #     assert len(self.attachments_robot) > 0, "No object attached!"
    #
    #     current_conf = get_joint_positions(self.robot, self.ik_joints)
    #     box_position, box_orientation = get_pose(object)
    #     logger.debug(f"box_position: {box_position}")
    #     logger.debug(f"box_orientation: {box_orientation}")
    #
    #     new_box_position = (x, y, z)
    #     # new_box_orientation = box_orientation
    #
    #     rot = Rotation.from_euler("XYZ", [np.pi, 0, theta], degrees=False)
    #     quat = rot.as_quat()
    #
    #     position_offset = self.position_offset_dict[self.last_grasp_direction]
    #     if domain_name == "packing":
    #         ee_grasp_position = new_box_position + position_offset
    #     if domain_name == "blocksworld_pr":
    #         ee_grasp_position = new_box_position + position_offset - np.array([0.0, 0, 0.03])
    #     if domain_name == "kitchen":
    #         ee_grasp_position = new_box_position + position_offset
    #     ee_grasp_orientation = self.rotation_offset_dict[self.last_grasp_direction]
    #     # ee_grasp_orientation = quat
    #
    #     print("Placing")
    #     print(f"ee_place_position of {ee_grasp_position}", f"ee_place_orientation of {ee_grasp_orientation}")
    #     print("attachments_robot:", self.attachments_robot)
    #     print("current conf:", current_conf)
    #
    #     if traj is None:
    #         success, traj, feedback = self.motion_planning(
    #             domain_name,
    #             self.robot,
    #             self.ik_joints,
    #             ee_grasp_position,
    #             ee_grasp_orientation,
    #             obstacles,
    #             self.attachments_robot,
    #             self.joint_limits,
    #         )
    #     else:
    #         assert (
    #             traj[0] == current_conf
    #         ), f"The start conf of the known trajectory {traj[0]} is not the same as the robot start conf {current_conf}"
    #
    #         success = True
    #         feedback = "Success"
    #
    #     # release gripper
    #     if success:
    #         ee_link_from_tcp = Pose(point=(0, 0.00, 0.00))
    #         # simulate action
    #         self.simulate_traj(
    #             self.robot,
    #             self.ik_joints,
    #             self.attachments_robot,
    #             object,
    #             self.tool_attach_link,
    #             ee_link_from_tcp,
    #             traj,
    #             play_traj,
    #         )
    #
    #         self.release_gripper()
    #         logger.debug(f"Placed object {object}!")
    #
    #     return success, traj, feedback

    def place(self, domain_name, object, obstacles, x, y, z, theta, traj=None, play_traj=True):
        assert len(self.attachments_robot) > 0, "No object attached!"

        # --- current state & target spec ---
        current_conf = get_joint_positions(self.robot, self.ik_joints)
        box_position, box_orientation = get_pose(object)
        logger.debug(f"box_position: {box_position}")
        logger.debug(f"box_orientation: {box_orientation}")

        new_box_position = np.array([x, y, z], dtype=float)
        if domain_name == "kitchen":
            rot = Rotation.from_euler("XYZ", [np.pi, 0, theta], degrees=False)
            default_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif domain_name == "blocksworld_pr":
            rot = Rotation.from_euler("XYZ", [theta, np.pi/2, 0], degrees=False)
            default_conf = TOP_HOLDING_LEFT_ARM
        quat = rot.as_quat()

        position_offset = self.position_offset_dict[self.last_grasp_direction]
        if domain_name == "packing":
            ee_place_pos = new_box_position + position_offset
        elif domain_name == "blocksworld_pr":
            ee_place_pos = new_box_position + position_offset - np.array([0.0, 0.0, 0.03])
        elif domain_name == "kitchen":
            ee_place_pos = new_box_position - np.array([0.0, 0.0, 0.03]) #+ position_offset

        # ee_place_ori = self.rotation_offset_dict[self.last_grasp_direction]
        ee_place_ori = quat

        # --- pre-place & lift (retreat) ---
        # pre grasp
        ee_pre_pos = (np.asarray(ee_place_pos) + self.pregrasp_offset * np.asarray([0, 0, 1])).tolist()
        ee_pre_ori = ee_place_ori

        # lift
        ee_lift_pos = (np.asarray(ee_place_pos) + self.lift_offset * np.asarray([0, 0, 1])).tolist()
        ee_lift_ori = ee_place_ori

        print("Placing")
        print(f"ee_pre_position: {ee_pre_pos}, ee_pre_orientation: {ee_pre_ori}")
        print(f"ee_place_position: {ee_place_pos}, ee_place_orientation: {ee_place_ori}")
        print("attachments_robot:", self.attachments_robot)
        print("current conf:", current_conf)

        ee_link_from_tcp = Pose(point=(0, 0.00, 0.00))

        if traj is None:
            # ---- 1) PRE-PLACE: current -> pre-place ----
            print("MOVE TO PRE PLACE POSE")
            ok1, path1, fb1 = self.motion_planning(
                domain_name,
                self.robot,
                self.ik_joints,
                ee_pre_pos,
                ee_pre_ori,
                obstacles,
                self.attachments_robot,
                self.joint_limits,
            )
            if not ok1:
                return False, None, f"Failed to reach pre-place: {fb1}"

            # set_joint_positions(self.robot, self.ik_joints, path1[-1])
            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                path1,
                play_traj,
            )

            # ---- 2) PLACE: pre-place -> place ----
            if domain_name == "kitchen":
                filtered = {k: v for k, v in obstacles.items() if v not in ["stove", "sink"]}
            else:
                filtered = obstacles
            print("MOVE CLOSER")
            ok2, path2, fb2 = self.motion_planning(
                domain_name,
                self.robot,
                self.ik_joints,
                ee_place_pos,
                ee_place_ori,
                filtered,
                self.attachments_robot,
                self.joint_limits,
            )
            if not ok2:
                set_joint_positions(self.robot, self.ik_joints, current_conf)
                return False, None, f"Failed to reach place pose: {fb2}"

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                path2,
                play_traj,
            )

            # ---- 3) RELEASE ----
            print("RELEASE GRIPPER")
            self.release_gripper()
            logger.debug(f"Placed object {object}!")
            for _ in range(3):
                p.stepSimulation()

            # set_joint_positions(self.robot, self.ik_joints, path_12[-1])

            # ---- 4) LIFT (RETREAT): place -> lift ----
            print("LIFT")
            ok3, path3, fb3 = self.motion_planning(
                domain_name,
                self.robot,
                self.ik_joints,
                ee_lift_pos,
                ee_lift_ori,
                obstacles,
                self.attachments_robot,
                self.joint_limits,
            )
            if not ok3:
                return False, None, f"Failed to retreat: {fb3}"

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                path3,
                play_traj,
            )

            print("MOVE THE DEFAULT POSE")
            path4, fb4 = plan_joint_motion(
                self.robot,
                self.ik_joints,
                default_conf,
                obstacles=list(obstacles.keys()),
                attachments=self.attachments_robot,
                self_collisions=False,
                custom_limits=self.joint_limits,
                diagnosis=False,
            )
            if path4 is None:
                logger.debug(fb4)
                set_joint_positions(self.robot, self.ik_joints, current_conf)
                return False, None, f"Failed to move to rest pose after grasp: {fb4}"

            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                path4,
                play_traj,
            )

            # 전체 경로
            traj = [path1, path2, path3, path4]
            success = True
            feedback = "Success"

        else:
            assert (
                    traj[0][0] == current_conf
            ), f"The start conf of the known trajectory {traj[0][0]} is not the same as the robot start conf {current_conf}"
            success = True
            feedback = "Success"

            # simulate known traj with attachment, then release
            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[0],
                play_traj,
            )
            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                object,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[1],
                play_traj,
            )
            self.release_gripper()
            logger.debug(f"Placed object {object}!")
            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[2],
                play_traj,
            )
            self.simulate_traj(
                self.robot,
                self.ik_joints,
                self.attachments_robot,
                None,
                self.tool_attach_link,
                ee_link_from_tcp,
                traj[3],
                play_traj,
            )

        return success, traj, feedback

    def release_gripper(self):
        self.attachments_robot = []
        self.last_grasp_direction = None

    def verify_ik(self, targeted_pos, targeted_ori, joint_conf):
        if joint_conf is None:
            return False
        original_conf = get_joint_positions(self.robot, self.ik_joints)
        print("self.ik_joints: ", self.ik_joints)
        print("joint_conf:", joint_conf)
        set_joint_positions(self.robot, self.ik_joints, joint_conf)
        pos, ori = get_link_pose(self.robot, self.tool_attach_link)

        dist = get_distance(pos, targeted_pos)
        ori_dist = quat_angle_between(ori, targeted_ori)
        # print("targeted_pos: ", targeted_pos)
        # print("targeted_ori: ", targeted_ori)
        # print("calculated pos:", pos)
        # print("calculated ori:", ori)
        # print("dist:", dist)
        # print("ori_dist:", ori_dist)

        set_joint_positions(self.robot, self.ik_joints, original_conf)

        if dist < 0.01 and ori_dist < 0.4:
            # print("IK solution found")
            return True
        else:
            # print("IK solution not found")
            return False

    def motion_planning(
        self,
        domain_name,
        robot,
        ik_joints,
        robot_ee_position,
        robot_end_orientation,
        obstacles,
        attachments_robot,
        custom_limits,
        diagnosis=False,
    ):
        # test ik first
        # end_conf = p.calculateInverseKinematics(
        #     bodyUniqueId=self.robot,
        #     endEffectorLinkIndex=self.tool_attach_link,
        #     targetPosition=robot_ee_position,
        #     targetOrientation=robot_end_orientation,
        #     # currentPosition=[0, -0.785398163397, 0, -2.35619449, 0, 1.57079632679, 0.78539816, 0.01, 0.01],
        #     maxNumIterations=100000,
        #     residualThreshold=0.0001,
        # )
        start_conf_snapshot = get_joint_positions(self.robot, self.ik_joints)

        if domain_name == 'blocksworld_pr':
            end_conf = pr2_ik(robot, 'left', [robot_ee_position, robot_end_orientation], custom_limits)
        elif domain_name == 'kitchen':
            # end_conf = inverse_kinematics(robot, self.tool_attach_link, [robot_ee_position, robot_end_orientation])
            end_conf = kuka_ik(robot, [robot_ee_position, robot_end_orientation], custom_limits)
        print("end_conf: ", end_conf)
        print("attachments:", attachments_robot)


        # ik_joints_idx = [0, 1, 2, 6,
        #                  7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 22, 23,
        #                  50, 52, 53, 54, 56, 57, 59, 60, 65, 66, 67, 68, 69, 70, 71,
        #                  74, 75, 76, 78, 79, 81, 82, 87, 88, 89, 90, 91, 92, 93, 96]
        #
        # index_to_position = {joint: i for i, joint in enumerate(ik_joints_idx)}
        # target_joints = [74, 75, 76, 78, 79, 81, 82]
        # filtered_conf = tuple(end_conf[index_to_position[j]] for j in target_joints)
        # print("calculated left arm ik: ", filtered_conf)

        ik_found = self.verify_ik(robot_ee_position, robot_end_orientation, end_conf)
        if not ik_found:
            logger.debug("Do not have a feasible ik")
            failure_feedback = "Failed because no IK solution exists for the grasping pose"
            return False, None, failure_feedback

        # test final ee pose if in collision
        if_collision, feedback = check_ee_collision(
            self.robot,
            self.ik_joints,
            self.tool_attach_link,
            end_conf,
            obstacles,
            self.attachments_robot,
        )
        if if_collision:
            logger.debug(feedback)
            return False, None, feedback
        set_joint_positions(self.robot, self.ik_joints, start_conf_snapshot)

        # plan motion
        logger.debug(str(obstacles))
        path, feedback = plan_joint_motion(
            robot,
            ik_joints,
            end_conf,
            obstacles=list(obstacles.keys()),
            attachments=attachments_robot,
            self_collisions=False,
            custom_limits=custom_limits,
            diagnosis=diagnosis,
        )
        # print("trajectory: ", path)
        # print("start of trajectory: ", path[0])
        # print("end of trajectory: ", path[-1])

        # process planned results
        if path is None:
            logger.debug(feedback)
            return False, None, feedback
        else:
            logger.debug("Found feasible motion plan!")

        return True, path, "Success"

    def simulate_traj(
        self,
        robot,
        ik_joints,
        attachments_robot,
        attached,
        tool_attach_link,
        ee_link_from_tcp,
        traj,
        play_traj,
    ):
        if play_traj:
            logger.debug("Simulate trajectory!")
            time_step = 0.03
            logger.debug(f"attachments_robot: {attachments_robot}")
            logger.debug(f"attached: {attached}")
            for conf in traj:
                set_joint_positions(robot, ik_joints, conf)
                if len(attachments_robot) > 0:
                    ee_link_pose = get_link_pose(robot, tool_attach_link)
                    set_pose(attached, multiply(ee_link_pose, ee_link_from_tcp))
                wait_for_duration(time_step)
        else:
            # directly set end conf
            set_joint_positions(robot, ik_joints, traj[-1])
            if len(attachments_robot) > 0:
                ee_link_pose = get_link_pose(robot, tool_attach_link)
                set_pose(attached, multiply(ee_link_pose, ee_link_from_tcp))
