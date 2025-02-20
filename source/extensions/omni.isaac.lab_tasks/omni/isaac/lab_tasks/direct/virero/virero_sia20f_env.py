from __future__ import annotations

import torch
from typing import Union

from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.torch.transformations import tf_combine, tf_inverse, tf_vector
from pxr import UsdGeom

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from omni.isaac.lab.envs import DirectRLEnv, DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.utils.math import sample_uniform

@configclass
class VireroSia20fEnvCfg(DirectRLEnvCfg):
    ############# todo: to edit
    # env
    episode_length_s = 8.3333  # 500 timesteps
    decimation = 2
    action_space = 9
    observation_space = 23
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120,
        render_interval=decimation,
        disable_contact_processing=True,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=3.0, replicate_physics=True)

    ######################################## till here
    
    # virero_env
    
    # todo: check if articulation cfg is correct?? since the root joint is not directly an articulation of the top parent.
    # todo later: try removing the articulation root from the root joint, it behaves well in the virero_ros file when added as reference.
    virero = ArticulationCfg(
        prim_path="/World/envs/env_.*/virero",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"/home/faps_ubuntu22/virero/virero_usd_files/virero_urdf-usd/
                        versuchsanlage_with_sia20f.usd",
        ),
        # Defaults to identity pose with zero velocity and zero joint state. check again later
        # init_state=ArticulationCfg.InitialStateCfg(
        #     pos=(0.0, 0, 0.4),
        #     rot=(0.1, 0.0, 0.0, 0.0),
        #     joint_pos={
        #         "door_left_joint": 0.0,
        #         "door_right_joint": 0.0,
        #         "drawer_bottom_joint": 0.0,
        #         "drawer_top_joint": 0.0,
        #     },
        # ),
        ####### todo: Check if actuators are really required?? if 
        # actuators={
        #     "drawers": ImplicitActuatorCfg(
        #         joint_names_expr=["drawer_top_joint", "drawer_bottom_joint"],
        #         effort_limit=87.0,
        #         velocity_limit=100.0,
        #         stiffness=10.0,
        #         damping=1.0,
        #     ),
        #     "doors": ImplicitActuatorCfg(
        #         joint_names_expr=["door_left_joint", "door_right_joint"],
        #         effort_limit=87.0,
        #         velocity_limit=100.0,
        #         stiffness=10.0,
        #         damping=2.5,
        #     ),
        #},
    )    
    
    # sia20f : configuring it as an articulation, but already spawned beforehand in the virero setup.
    sia20f = ArticulationCfg(
        prim_path="/World/envs/env_.*/virero/sia20f",
        # todo: check if it works without spawn
        # spawn=sim_utils.UsdFileCfg(
        #     usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Franka/franka_instanceable.usd",
        #     activate_contact_sensors=False,
        #     rigid_props=sim_utils.RigidBodyPropertiesCfg(
        #         disable_gravity=False,
        #         max_depenetration_velocity=5.0,
        #     ),
        #     articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        #         enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=1
        #     ),
        # ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "sia20f_linearachse_inner_joint": 0.0,
                "sia20f_linearachse_middle_joint": 0.0,
                "sia20f_joint_1_s": -0.363,
                "sia20f_joint_2_l": 0.3159,
                "sia20f_joint_3_e": 1.3491,
                "sia20f_joint_4_u": -0.8762,
                "sia20f_joint_5_r": -0.0593,
                "sia20f_joint_6_b": -0.7976,
                "sia20f_joint_7_t": -0.0297,
            },
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        actuators={
            "sia20f_all": ImplicitActuatorCfg(
                joint_names_expr=[
                    "sia20f_linearachse_inner_joint",
                    "sia20f_linearachse_middle_joint",
                    "sia20f_joint_1_s",
                    "sia20f_joint_2_l",
                    "sia20f_joint_3_e",
                    "sia20f_joint_4_u",
                    "sia20f_joint_5_r",
                    "sia20f_joint_6_b",
                    "sia20f_joint_7_t",
                ],
                ## todo: to check if the values need to be changed???
                effort_limit=None,
                velocity_limit=None,
                stiffness=None,
                damping=None,
            ),
            # "panda_forearm": ImplicitActuatorCfg(
            #     joint_names_expr=["panda_joint[5-7]"],
            #     effort_limit=12.0,
            #     velocity_limit=2.61,
            #     stiffness=80.0,
            #     damping=4.0,
            # ),
            # "panda_hand": ImplicitActuatorCfg(
            #     joint_names_expr=["panda_finger_joint.*"],
            #     effort_limit=200.0,
            #     velocity_limit=0.2,
            #     stiffness=2e3,
            #     damping=1e2,
            # ),
        },
    )
    

    # Set each stacking cube deterministically
    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube_1",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.4, 0.0, 0.0203), rot=(1, 0, 0, 0)),
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
        ),
    )

    # todo: check ground plane values later
    # ground plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    ############# todo: check following values
    action_scale = 7.5
    dof_velocity_scale = 0.1

    # reward scales
    dist_reward_scale = 1.5
    rot_reward_scale = 1.5
    open_reward_scale = 10.0
    action_penalty_scale = 0.05
    finger_reward_scale = 2.0
    ############### till here


class VireroSia20fEnv(DirectRLEnv):
    # pre-physics step calls
    #   |-- _pre_physics_step(action)
    #   |-- _apply_action()
    # post-physics step calls
    #   |-- _get_dones()
    #   |-- _get_rewards()
    #   |-- _reset_idx(env_ids)
    #   |-- _get_observations()

    cfg: VireroSia20fEnvCfg

    def __init__(self, cfg: VireroSia20fEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        def get_env_local_pose(env_pos: torch.Tensor, xformable: UsdGeom.Xformable, device: Union[str, torch.device, int]):
            """Compute pose in env-local coordinates"""
            world_transform = xformable.ComputeLocalToWorldTransform(0)
            world_pos = world_transform.ExtractTranslation()
            world_quat = world_transform.ExtractRotationQuat()

            px = world_pos[0] - env_pos[0]
            py = world_pos[1] - env_pos[1]
            pz = world_pos[2] - env_pos[2]
            qx = world_quat.imaginary[0]
            qy = world_quat.imaginary[1]
            qz = world_quat.imaginary[2]
            qw = world_quat.real

            return torch.tensor([px, py, pz, qw, qx, qy, qz], device=device)

        self.dt = self.cfg.sim.dt * self.cfg.decimation

        # create auxiliary variables for computing applied action, observations and rewards
        self.sia20f_dof_lower_limits = self._sia20f.data.soft_joint_pos_limits[0, :, 0].to(device=self.device)
        self.sia20f_dof_upper_limits = self._sia20f.data.soft_joint_pos_limits[0, :, 1].to(device=self.device)

        # todo: adjust the speed limits later
        self.robot_dof_speed_scales = torch.ones_like(self.sia20f_dof_lower_limits)
        
        # todo: add robotiq fingers
        # self.robot_dof_speed_scales[self._robot.find_joints("panda_finger_joint1")[0]] = 0.1
        # self.robot_dof_speed_scales[self._robot.find_joints("panda_finger_joint2")[0]] = 0.1

        self.robot_dof_targets = torch.zeros((self.num_envs, self._sia20f.num_joints), device=self.device)

        stage = get_current_stage()
        hand_pose = get_env_local_pose(
            self.scene.env_origins[0],
            UsdGeom.Xformable(stage.GetPrimAtPath("/World/envs/env_0/virero/sia20f/sia20f_joint_7_t")),
            self.device,
        )
        # todo: later after adding the robotiq gripper
        # lfinger_pose = get_env_local_pose(
        #     self.scene.env_origins[0],
        #     UsdGeom.Xformable(stage.GetPrimAtPath("/World/envs/env_0/Robot/panda_leftfinger")),
        #     self.device,
        # )
        # rfinger_pose = get_env_local_pose(
        #     self.scene.env_origins[0],
        #     UsdGeom.Xformable(stage.GetPrimAtPath("/World/envs/env_0/Robot/panda_rightfinger")),
        #     self.device,
        # )

        # edit: added cube local pose
        cube_local_pose = get_env_local_pose(
            self.scene.env_origins[0],
            UsdGeom.Xformable(stage.GetPrimAtPath("/World/envs/env_0/Cube_1")),
            self.device,
        )
        
        cube_local_inv_rot, cube_local_inv_pos = tf_inverse(cube_local_pose[3:7], cube_local_pose[0:3])
        self.cube_local_pos = cube_local_inv_pos.repeat((self.num_envs, 1))
        self.cube_local_rot = cube_local_inv_rot.repeat((self.num_envs, 1))
    
        # todo: later after adding the robotiq gripper
        # finger_pose = torch.zeros(7, device=self.device)
        # finger_pose[0:3] = (lfinger_pose[0:3] + rfinger_pose[0:3]) / 2.0
        # finger_pose[3:7] = lfinger_pose[3:7]
        hand_pose_inv_rot, hand_pose_inv_pos = tf_inverse(hand_pose[3:7], hand_pose[0:3])

        # todo: later change the below command
        # robot_local_grasp_pose_rot, robot_local_pose_pos = tf_combine(
        #     hand_pose_inv_rot, hand_pose_inv_pos, finger_pose[3:7], finger_pose[0:3]
        # )

        # done: modified the below 3 commands from  robot_local_pose_pos & robot_local_grasp_pose_rot to hand_pose_inv_pos & hand_pose_inv_rot
        hand_pose_inv_pos += torch.tensor([0, 0.04, 0], device=self.device)
        self.robot_local_grasp_pos = hand_pose_inv_pos.repeat((self.num_envs, 1))  #### todo: repeat does not seem to be recogniseing it as a tensor??? check later
        self.robot_local_grasp_rot = hand_pose_inv_rot.repeat((self.num_envs, 1))
        
        # todo: later after adding the robotiq gripper
        # drawer_local_grasp_pose = torch.tensor([0.3, 0.01, 0.0, 1.0, 0.0, 0.0, 0.0], device=self.device)
        # self.drawer_local_grasp_pos = drawer_local_grasp_pose[0:3].repeat((self.num_envs, 1))
        # self.drawer_local_grasp_rot = drawer_local_grasp_pose[3:7].repeat((self.num_envs, 1))

        # self.gripper_forward_axis = torch.tensor([0, 0, 1], device=self.device, dtype=torch.float32).repeat(
        #     (self.num_envs, 1)
        # )
        # self.drawer_inward_axis = torch.tensor([-1, 0, 0], device=self.device, dtype=torch.float32).repeat(
        #     (self.num_envs, 1)
        # )
        # self.gripper_up_axis = torch.tensor([0, 1, 0], device=self.device, dtype=torch.float32).repeat(
        #     (self.num_envs, 1)
        # )
        # self.drawer_up_axis = torch.tensor([0, 0, 1], device=self.device, dtype=torch.float32).repeat(
        #     (self.num_envs, 1)
        # )

        self.hand_link_idx = self._sia20f.find_bodies("sia20f_joint_7_t")[0][0]
        # todo: later after adding the robotiq gripper
        # self.left_finger_link_idx = self._robot.find_bodies("panda_leftfinger")[0][0]
        # self.right_finger_link_idx = self._robot.find_bodies("panda_rightfinger")[0][0]
        # self.drawer_link_idx = self._cabinet.find_bodies("drawer_top")[0][0]

        self.robot_grasp_rot = torch.zeros((self.num_envs, 4), device=self.device)
        self.robot_grasp_pos = torch.zeros((self.num_envs, 3), device=self.device)
        # self.drawer_grasp_rot = torch.zeros((self.num_envs, 4), device=self.device)
        # self.drawer_grasp_pos = torch.zeros((self.num_envs, 3), device=self.device)
        
        # edit: added cube position and rotation
        self.cube_pos = torch.tensor((self.num_envs, 3), device=self.device)
        self.cube_rot = torch.tensor((self.num_envs, 4), device=self.device)

    def _setup_scene(self):
        self._sia20f = Articulation(self.cfg.sia20f)
        self._virero = Articulation(self.cfg.virero)
        self._cube = RigidObject(self.cfg.cube)
        self.scene.articulations["sia20f"] = self._sia20f
        self.scene.articulations["virero"] = self._virero
        self.scene.rigid_objects["cube"] = self._cube

        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        # clone, filter, and replicate
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        # todo: later check ... add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    # pre-physics step calls

    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.clone().clamp(-1.0, 1.0)
        targets = self.robot_dof_targets + self.robot_dof_speed_scales * self.dt * self.actions * self.cfg.action_scale
        self.robot_dof_targets[:] = torch.clamp(targets, self.sia20f_dof_lower_limits, self.sia20f_dof_upper_limits)

    def _apply_action(self):
        self._sia20f.set_joint_position_target(self.robot_dof_targets)

    # post-physics step calls

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # terminated = self._cabinet.data.joint_pos[:, 3] > 0.39
        # todo: check termination condition
        terminated = self._sia20f.data.joint_pos[:] > self.sia20f_dof_upper_limits[0] - 0.01
        truncated = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, truncated

    def _get_rewards(self) -> torch.Tensor:
        # Refresh the intermediate values after the physics steps
        self._compute_intermediate_values()
        # todo: later after adding the robotiq gripper
        # robot_left_finger_pos = self._robot.data.body_link_pos_w[:, self.left_finger_link_idx]
        # robot_right_finger_pos = self._robot.data.body_link_pos_w[:, self.right_finger_link_idx]

        # return self._compute_rewards(
        #     self.actions,
        #     self._cabinet.data.joint_pos,
        #     self.robot_grasp_pos,
        #     self.drawer_grasp_pos,
        #     self.robot_grasp_rot,
        #     self.drawer_grasp_rot,
        #     robot_left_finger_pos,
        #     robot_right_finger_pos,
        #     self.gripper_forward_axis,
        #     self.drawer_inward_axis,
        #     self.gripper_up_axis,
        #     self.drawer_up_axis,
        #     self.num_envs,
        #     self.cfg.dist_reward_scale,
        #     self.cfg.rot_reward_scale,
        #     self.cfg.open_reward_scale,
        #     self.cfg.action_penalty_scale,
        #     self.cfg.finger_reward_scale,
        #     self._robot.data.joint_pos,
        # )
        return self._compute_rewards()

    def _reset_idx(self, env_ids: torch.Tensor | None):
        super()._reset_idx(env_ids)
        # robot state
        joint_pos = self._sia20f.data.default_joint_pos[env_ids] + sample_uniform(
            -0.125,
            0.125,
            (len(env_ids), self._sia20f.num_joints),
            self.device,
        )
        joint_pos = torch.clamp(joint_pos, self.sia20f_dof_lower_limits, self.sia20f_dof_upper_limits)
        joint_vel = torch.zeros_like(joint_pos)
        self._sia20f.set_joint_position_target(joint_pos, env_ids=env_ids)
        self._sia20f.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

        # # cabinet state
        # zeros = torch.zeros((len(env_ids), self._cabinet.num_joints), device=self.device)
        # self._cabinet.write_joint_state_to_sim(zeros, zeros, env_ids=env_ids)
        
        # cube state
        cube_pos = torch.tensor([0.4, 0.0, 0.0203], device=self.device).repeat((len(env_ids), 1))
        cube_rot = torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).repeat((len(env_ids), 1))
        # todo: check the correct command for setting the state
        self._cube.set_state(cube_pos, cube_rot, env_ids=env_ids)

        
        # Need to refresh the intermediate values so that _get_observations() can use the latest values
        self._compute_intermediate_values(env_ids)

    def _get_observations(self) -> dict:
        # todo: check why this scaled pos... seems to be normalised, but why 2.0 and -1.0
        dof_pos_scaled = (
            2.0
            * (self._sia20f.data.joint_pos - self.sia20f_dof_lower_limits)
            / (self.sia20f_dof_upper_limits - self.sia20f_dof_lower_limits)
            - 1.0
        )
        
        to_target = self.cube_pos - self.robot_grasp_pos

        obs = torch.cat(
            (
                dof_pos_scaled,
                self._sia20f.data.joint_vel * self.cfg.dof_velocity_scale,
                to_target,
                # self._cabinet.data.joint_pos[:, 3].unsqueeze(-1),
                # self._cabinet.data.joint_vel[:, 3].unsqueeze(-1),
            ),
            dim=-1,
        )
        return {"policy": torch.clamp(obs, -5.0, 5.0)}

    # auxiliary methods

    # edit: added the cube position and rotation calculation
    def _compute_intermediate_values(self, env_ids: torch.Tensor | None = None):
        if env_ids is None:
            env_ids = self._sia20f._ALL_INDICES

        hand_pos = self._sia20f.data.body_link_pos_w[env_ids, self.hand_link_idx]
        hand_rot = self._sia20f.data.body_link_quat_w[env_ids, self.hand_link_idx]
        # drawer_pos = self._cabinet.data.body_link_pos_w[env_ids, self.drawer_link_idx]
        # drawer_rot = self._cabinet.data.body_link_quat_w[env_ids, self.drawer_link_idx]
        # edit: added cube position and rotation
        cube_pos = self._cube.data.body_link_pos_w[env_ids, 0]
        cube_rot = self._cube.data.body_link_quat_w[env_ids, 0]
        
        (
            self.robot_grasp_rot[env_ids],
            self.robot_grasp_pos[env_ids],
            self.cube_rot[env_ids],
            self.cube_pos[env_ids],
        ) = self._compute_grasp_transforms(
            hand_rot,
            hand_pos,
            self.robot_local_grasp_rot[env_ids],
            self.robot_local_grasp_pos[env_ids],
            cube_rot,
            cube_pos,
            self.cube_local_rot[env_ids],
            self.cube_local_pos[env_ids],
        )

    def _compute_rewards(self) -> torch.Tensor:
        # distance from hand to the drawer
        d = torch.linalg.vector_norm(self.robot_grasp_pos - self.cube_pos, p=2, dim=-1)  # todo: verify if vector_norm is correct, change to matrix_norm?
        scale = 200
        sensitivity = 1
        dist_reward = - (scale * torch.tanh(sensitivity * d))

        # # regularization on the actions (summed for each environment)
        # action_penalty = torch.sum(self.actions ** 2, dim=-1)

        # per step penalty
        penalty_step = torch.full_like(dist_reward, (0.5 / 2000))
        
        return (dist_reward + penalty_step)
    
    # def _compute_rewards(
    #     self,
    #     actions,
    #     cabinet_dof_pos,
    #     franka_grasp_pos,
    #     drawer_grasp_pos,
    #     franka_grasp_rot,
    #     drawer_grasp_rot,
    #     franka_lfinger_pos,
    #     franka_rfinger_pos,
    #     gripper_forward_axis,
    #     drawer_inward_axis,
    #     gripper_up_axis,
    #     drawer_up_axis,
    #     num_envs,
    #     dist_reward_scale,
    #     rot_reward_scale,
    #     open_reward_scale,
    #     action_penalty_scale,
    #     finger_reward_scale,
    #     joint_positions,
    # ):
    #     # distance from hand to the drawer
    #     d = torch.norm(franka_grasp_pos - drawer_grasp_pos, p=2, dim=-1)
    #     dist_reward = 1.0 / (1.0 + d**2)
    #     dist_reward *= dist_reward
    #     dist_reward = torch.where(d <= 0.02, dist_reward * 2, dist_reward)

    #     axis1 = tf_vector(franka_grasp_rot, gripper_forward_axis)
    #     axis2 = tf_vector(drawer_grasp_rot, drawer_inward_axis)
    #     axis3 = tf_vector(franka_grasp_rot, gripper_up_axis)
    #     axis4 = tf_vector(drawer_grasp_rot, drawer_up_axis)

    #     dot1 = (
    #         torch.bmm(axis1.view(num_envs, 1, 3), axis2.view(num_envs, 3, 1)).squeeze(-1).squeeze(-1)
    #     )  # alignment of forward axis for gripper
    #     dot2 = (
    #         torch.bmm(axis3.view(num_envs, 1, 3), axis4.view(num_envs, 3, 1)).squeeze(-1).squeeze(-1)
    #     )  # alignment of up axis for gripper
    #     # reward for matching the orientation of the hand to the drawer (fingers wrapped)
    #     rot_reward = 0.5 * (torch.sign(dot1) * dot1**2 + torch.sign(dot2) * dot2**2)

    #     # regularization on the actions (summed for each environment)
    #     action_penalty = torch.sum(actions**2, dim=-1)

    #     # how far the cabinet has been opened out
    #     open_reward = cabinet_dof_pos[:, 3]  # drawer_top_joint

    #     # penalty for distance of each finger from the drawer handle
    #     lfinger_dist = franka_lfinger_pos[:, 2] - drawer_grasp_pos[:, 2]
    #     rfinger_dist = drawer_grasp_pos[:, 2] - franka_rfinger_pos[:, 2]
    #     finger_dist_penalty = torch.zeros_like(lfinger_dist)
    #     finger_dist_penalty += torch.where(lfinger_dist < 0, lfinger_dist, torch.zeros_like(lfinger_dist))
    #     finger_dist_penalty += torch.where(rfinger_dist < 0, rfinger_dist, torch.zeros_like(rfinger_dist))

    #     rewards = (
    #         dist_reward_scale * dist_reward
    #         + rot_reward_scale * rot_reward
    #         + open_reward_scale * open_reward
    #         + finger_reward_scale * finger_dist_penalty
    #         - action_penalty_scale * action_penalty
    #     )

    #     self.extras["log"] = {
    #         "dist_reward": (dist_reward_scale * dist_reward).mean(),
    #         "rot_reward": (rot_reward_scale * rot_reward).mean(),
    #         "open_reward": (open_reward_scale * open_reward).mean(),
    #         "action_penalty": (-action_penalty_scale * action_penalty).mean(),
    #         "left_finger_distance_reward": (finger_reward_scale * lfinger_dist).mean(),
    #         "right_finger_distance_reward": (finger_reward_scale * rfinger_dist).mean(),
    #         "finger_dist_penalty": (finger_reward_scale * finger_dist_penalty).mean(),
    #     }

    #     # bonus for opening drawer properly
    #     rewards = torch.where(cabinet_dof_pos[:, 3] > 0.01, rewards + 0.25, rewards)
    #     rewards = torch.where(cabinet_dof_pos[:, 3] > 0.2, rewards + 0.25, rewards)
    #     rewards = torch.where(cabinet_dof_pos[:, 3] > 0.35, rewards + 0.25, rewards)

    #     return rewards

    def _compute_grasp_transforms(
        self,
        hand_rot,
        hand_pos,
        franka_local_grasp_rot,
        franka_local_grasp_pos,
        cube_rot,
        cube_pos,
        cube_local_grasp_rot,
        cube_local_grasp_pos,
    ):
        global_robot_rot, global_robot_pos = tf_combine(
            hand_rot, hand_pos, franka_local_grasp_rot, franka_local_grasp_pos
        )
        global_cube_rot, global_cube_pos = tf_combine(
            cube_rot, cube_pos, cube_local_grasp_rot, cube_local_grasp_pos
        )

        return global_robot_rot, global_robot_pos, global_cube_rot, global_cube_pos
