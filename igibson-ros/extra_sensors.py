import os
from collections import OrderedDict

import numpy as np
import igibson
from igibson.sensors.dropout_sensor_noise import DropoutSensorNoise
from igibson.sensors.sensor_base import BaseSensor
from igibson.utils.mesh_util import quat2rotmat, xyzw2wxyz


class ExtraVisionSensor(BaseSensor):
    def __init__(
            self, 
            env,
            modalities, 
            camera_pos,
            camera_orn
        ):
        # ExtraVisionSensor inits the base class with env,
        # but mostly does not use the existing env config
        super(ExtraVisionSensor, self).__init__(env)

        # Setting the config separately
        self.modalities = modalities
        self.raw_modalities = self.get_raw_modalities(modalities)
        self.image_width = self.config.get("image_width", 128)
        self.image_height = self.config.get("image_height", 128)

        self.depth_noise_rate = self.config.get("depth_noise_rate", 0.0)
        self.depth_low = self.config.get("depth_low", 0.5)
        self.depth_high = self.config.get("depth_high", 5.0)

        self.camera_pos = camera_pos
        self.camera_orn = quat2rotmat(xyzw2wxyz(camera_orn))[:3, :3]

        self.noise_model = DropoutSensorNoise(env)
        self.noise_model.set_noise_rate(self.depth_noise_rate)
        self.noise_model.set_noise_value(0.0)

    def get_raw_modalities(self, modalities):
        """
        Helper function that gathers raw modalities (e.g. depth is based on 3d)
        :return: raw modalities to query the renderer
        """
        raw_modalities = []
        if "rgb" in modalities or "highlight" in modalities:
            raw_modalities.append("rgb")
        if "depth" in modalities or "pc" in modalities:
            raw_modalities.append("3d")
        if "seg" in modalities:
            raw_modalities.append("seg")
        if "ins_seg" in modalities:
            raw_modalities.append("ins_seg")
        if "normal" in modalities:
            raw_modalities.append("normal")
        if "optical_flow" in modalities:
            raw_modalities.append("optical_flow")
        if "scene_flow" in modalities:
            raw_modalities.append("scene_flow")
        return raw_modalities


    def get_rgb(self, raw_vision_obs):
        """
        :return: RGB sensor reading, normalized to [0.0, 1.0]
        """
        return raw_vision_obs["rgb"][:, :, :3]

    def get_highlight(self, raw_vision_obs):
        if not "rgb" in raw_vision_obs:
            raise ValueError("highlight depends on rgb")

        return (raw_vision_obs["rgb"][:, :, 3:4] > 0).astype(np.float32)

    def get_depth(self, raw_vision_obs):
        """
        :return: depth sensor reading, normalized to [0.0, 1.0]
        """
        depth = -raw_vision_obs["3d"][:, :, 2:3]
        # 0.0 is a special value for invalid entries
        depth[depth < self.depth_low] = 0.0
        depth[depth > self.depth_high] = 0.0

        # re-scale depth to [0.0, 1.0]
        depth /= self.depth_high
        depth = self.noise_model.add_noise(depth)

        return depth

    def get_pc(self, raw_vision_obs):
        """
        :return: pointcloud sensor reading
        """
        return raw_vision_obs["3d"][:, :, :3]

    def get_optical_flow(self, raw_vision_obs):
        """
        :return: optical flow sensor reading
        """
        return raw_vision_obs["optical_flow"][:, :, :2]

    def get_scene_flow(self, raw_vision_obs):
        """
        :return: scene flow sensor reading
        """
        return raw_vision_obs["scene_flow"][:, :, :3]

    def get_normal(self, raw_vision_obs):
        """
        :return: surface normal reading
        """
        return raw_vision_obs["normal"][:, :, :3]

    def get_seg(self, raw_vision_obs):
        """
        :return: semantic segmentation mask, between 0 and MAX_CLASS_COUNT
        """
        seg = np.round(raw_vision_obs["seg"][:, :, 0:1] * MAX_CLASS_COUNT).astype(np.int32)
        return seg

    def get_ins_seg(self, raw_vision_obs):
        """
        :return: semantic segmentation mask, between 0 and MAX_INSTANCE_COUNT
        """
        seg = np.round(raw_vision_obs["ins_seg"][:, :, 0:1] * MAX_INSTANCE_COUNT).astype(np.int32)
        return seg

    def get_obs(self, env):
        """
        Get vision sensor reading
        :return: vision sensor reading
        """
        raw_vision_obs = []

        default_sensor_pos = env.robots[0].eyes.get_position()
        default_sensor_orn = env.robots[0].eyes.get_orientation()
        default_sensor_mat = quat2rotmat(xyzw2wxyz(default_sensor_orn))[:3, :3]

        cam_pos = default_sensor_mat.dot(self.camera_pos) + default_sensor_pos 
        cam_orn = default_sensor_mat.dot(self.camera_orn)
        view_dir = cam_orn.dot(np.array([1, 0, 0]))
        env.simulator.renderer.set_camera(cam_pos, cam_pos + view_dir, [0, 0, 1], cache=True)
        for item in env.simulator.renderer.render(modes=self.raw_modalities):
            raw_vision_obs.append(item)

        raw_vision_obs = {mode: value for mode, value in zip(self.raw_modalities, raw_vision_obs)}

        vision_obs = OrderedDict()
        if "rgb" in self.modalities:
            vision_obs["rgb"] = self.get_rgb(raw_vision_obs)
        if "depth" in self.modalities:
            vision_obs["depth"] = self.get_depth(raw_vision_obs)
        if "pc" in self.modalities:
            vision_obs["pc"] = self.get_pc(raw_vision_obs)
        if "optical_flow" in self.modalities:
            vision_obs["optical_flow"] = self.get_optical_flow(raw_vision_obs)
        if "scene_flow" in self.modalities:
            vision_obs["scene_flow"] = self.get_scene_flow(raw_vision_obs)
        if "normal" in self.modalities:
            vision_obs["normal"] = self.get_normal(raw_vision_obs)
        if "seg" in self.modalities:
            vision_obs["seg"] = self.get_seg(raw_vision_obs)
        if "ins_seg" in self.modalities:
            vision_obs["ins_seg"] = self.get_ins_seg(raw_vision_obs)
        if "highlight" in self.modalities:
            vision_obs["highlight"] = self.get_highlight(raw_vision_obs)

        return vision_obs
