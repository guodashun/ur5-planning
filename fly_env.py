from sys import modules
import robosuite as suite
from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models import arenas
from robosuite.models.tasks import ManipulationTask
from robosuite.models.arenas import EmptyArena
from robosuite.models.objects import BallObject
from robosuite.utils.placement_samplers import UniformRandomSampler

class FlyEnv(SingleArmEnv):
    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        mount_types="default",
        gripper_types="default",
        initialization_noise="default",
        reward_scale=1.0,
        use_object_obs=False,
        placement_initializer=None,
        use_camera_obs=True,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera=None,
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
    ):
        
        self.reward_scale = reward_scale
        self.use_object_obs = use_object_obs
        
        # object placement initializer
        self.placement_initializer = placement_initializer

        super().__init__(robots, env_configuration=env_configuration, controller_configs=controller_configs, mount_types=mount_types, gripper_types=gripper_types, initialization_noise=initialization_noise, use_camera_obs=use_camera_obs, has_renderer=has_renderer, has_offscreen_renderer=has_offscreen_renderer, render_camera=render_camera, render_collision_mesh=render_collision_mesh, render_visual_mesh=render_visual_mesh, render_gpu_device_id=render_gpu_device_id, control_freq=control_freq, horizon=horizon, ignore_done=ignore_done, hard_reset=hard_reset, camera_names=camera_names, camera_heights=camera_heights, camera_widths=camera_widths, camera_depths=camera_depths)
    
    def _load_model(self):

        super()._load_model()

        # there may be a banana

        arena = EmptyArena()

        # sphere = BallObject(
        #     name="sphere",
        #     size=[0.04],
        #     rgba=[0, 0.5, 0.5, 1]
        # )

        # # Create placement initializer
        # if self.placement_initializer is not None:
        #     self.placement_initializer.reset()
        #     self.placement_initializer.add_objects(sphere)
        # else:
        #     self.placement_initializer = UniformRandomSampler(
        #         name="ObjectSampler",
        #         mujoco_objects=sphere,
        #         x_range=[-0.03, 0.03],
        #         y_range=[-0.03, 0.03],
        #         rotation=None,
        #         ensure_object_boundary_in_range=False,
        #         ensure_valid_placement=True,
        #         z_offset=0.01,
        #     )

        self.model = ManipulationTask(
            mujoco_arena=arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=None,
        )


    def reward(self, action):
        return 1.0
        # super().reward()

    def step(self, action):
        # add fly object traj
        return super().step(action)

    def reset(self):

        return super().reset()
        # add fly object traj
