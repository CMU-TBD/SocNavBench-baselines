from typing import Tuple

import numpy as np
from agents.agent import Agent
from obstacles.obstacle_map import ObstacleMap
from params.central_params import create_agent_params
from trajectory.trajectory import SystemConfig, Trajectory
from utils.utils import euclidean_dist2

from joystick_py.joystick_remote import JoystickRemote


class JoystickRVOwCkpt(JoystickRemote):
    def __init__(self):
        super().__init__(algo_name="RVO_w_ckpt", HOST="127.0.0.1", PORT=2111)

    def init_obstacle_map_ckpt(self) -> ObstacleMap:
        """ Initializes the sbpd map."""
        p = self.agent_params.obstacle_map_params
        env = self.current_ep.get_environment()
        return p.obstacle_map(
            p,
            renderer=0,  # ignoring rendering (dummy value)
            res=float(env["map_scale"]) * 100.0,
            map_trav=np.array(env["map_traversible"]),
        )

    def init_control_pipeline(self) -> None:
        super().init_control_pipeline()
        self.robot_goal_final = self.get_robot_goal()

        self.agent_params = create_agent_params(
            with_obstacle_map=True, with_planner=True
        )
        self.obstacle_map = self.init_obstacle_map_ckpt()
        self.obj_fn = Agent._init_obj_fn(self, params=self.agent_params)
        self.fmm_map = Agent._init_fmm_map(self, params=self.agent_params)
        self.planner = Agent._init_planner(self, params=self.agent_params)
        self.vehicle_data = self.planner.empty_data_dict()
        self.system_dynamics = Agent._init_system_dynamics(
            self, params=self.agent_params
        )
        self.robot_goal = self.robot

    def from_conf(self, configs: SystemConfig, idx: int) -> Tuple[float, float, float]:
        x = float(configs._position_nk2[0][idx][0])
        y = float(configs._position_nk2[0][idx][1])
        th = float(configs._heading_nk1[0][idx][0])
        return (x, y, th)

    def query_next_ckpt(self) -> Tuple[float, float, float]:
        robot_v_norm = np.sqrt(self.robot_v[0] ** 2 + self.robot_v[1] ** 2)
        robot_w = self.robot_v[2]
        robot_config = SystemConfig.from_pos3(
            self.robot, dt=self.agent_params.dt, v=robot_v_norm, w=robot_w
        )
        planner_data = self.planner.optimize(
            robot_config,
            self.goal_config,
            sim_state_hist=list(self.sim_states.values()),
        )
        tmp_goal = Trajectory.new_traj_clip_along_time_axis(
            planner_data["trajectory"],
            self.agent_params.control_horizon,
            repeat_second_to_last_speed=True,
        )
        robot_goal = self.from_conf(tmp_goal, -1)

        return robot_goal

    def convert_to_string(self) -> str:
        info_string = ""
        info_string += "delta_t\n"
        info_string += str(self.sim_dt) + "\n"
        info_string += "obstacles\n"
        for ed in self.environment:
            info_string += (
                str(ed[0][1])
                + ","
                + str(ed[0][0])
                + ","
                + str(ed[1][1])
                + ","
                + str(ed[1][0])
                + "\n"
            )
        info_string += "robot\n"
        goal_pos3 = self.goal_config.position_and_heading_nk3(squeeze=True)
        info_string += (
            str(self.robot[0])
            + ","
            + str(self.robot[1])
            + ","
            + str(self.robot[2])
            + ","
            + str(self.robot_v[0])
            + ","
            + str(self.robot_v[1])
            + ","
            + str(self.robot_v[2])
            + ","
            + str(goal_pos3[0])
            + ","
            + str(goal_pos3[1])
            + ","
            + str(goal_pos3[2])
            + ","
            + str(self.robot_radius * 1.05)
            + "\n"
        )
        info_string += "agents\n"
        for key in list(self.agents.keys()):
            agent = self.agents[key]
            agent_v = self.agents_v[key]
            agent_goal = self.agents_goals[key]
            agent_radius = self.agents_radius[key] * 1.05
            info_string += (
                key
                + ","
                + str(agent[0])
                + ","
                + str(agent[1])
                + ","
                + str(agent[2])
                + ","
                + str(agent_v[0])
                + ","
                + str(agent_v[1])
                + ","
                + str(agent_v[2])
                + ","
                + str(agent_goal[0])
                + ","
                + str(agent_goal[1])
                + ","
                + str(agent_goal[2])
                + ","
                + str(agent_radius)
                + "\n"
            )
        info_string += "End*"
        return info_string

    def joystick_sense(self) -> None:
        super().joystick_sense()

        ckpt_radius = 1
        goal_radius = 5
        if euclidean_dist2(self.robot, self.robot_goal) <= ckpt_radius:
            if euclidean_dist2(self.robot, self.robot_goal_final) < goal_radius:
                self.robot_goal = self.robot_goal_final
            else:
                self.robot_goal = self.query_next_ckpt()
