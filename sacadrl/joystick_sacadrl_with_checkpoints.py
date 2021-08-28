import numpy as np
from agents.agent import Agent as Ag
from params.central_params import create_agent_params
from trajectory.trajectory import SystemConfig, Trajectory
from utils.utils import euclidean_dist2

from sacadrl.joystick_sacadrl import (
    Agent,
    CADRLPolicy,
    CollisionAvoidanceEnv,
    JoystickSACADRL,
    LaserScanSensor,
    NonCooperativePolicy,
    OtherAgentsStatesSensor,
    UnicycleDynamics,
)


class JoystickSACADRLwCkpt(JoystickSACADRL):
    def __init__(self):
        super().__init__("sacadrl_w_ckpt")

    def init_obstacle_map_ckpt(self, renderer=0):
        """ Initializes the sbpd map."""
        p = self.agent_params.obstacle_map_params
        env = self.current_ep.get_environment()
        return p.obstacle_map(
            p,
            renderer,
            res=float(env["map_scale"]) * 100.0,
            map_trav=np.array(env["map_traversible"]),
        )

    def init_control_pipeline(self):
        self.robot_goal_final = self.get_robot_goal()
        self.goal_config = SystemConfig.from_pos3(self.robot_goal_final)
        env = self.current_ep.get_environment()
        self.init_obstacle_map(env)
        self.robot = self.get_robot_start()
        self.agents = {}
        agents_info = self.current_ep.get_agents()
        for key in list(agents_info.keys()):
            agent = agents_info[key]
            self.agents[key] = agent.get_current_config().position_and_heading_nk3(
                squeeze=True
            )

        self.agent_params = create_agent_params(
            with_planner=True, with_obstacle_map=True
        )
        self.obstacle_map = self.init_obstacle_map_ckpt()
        self.obj_fn = Ag._init_obj_fn(self, params=self.agent_params)
        self.fmm_map = Ag._init_fmm_map(self, params=self.agent_params)
        self.planner = Ag._init_planner(self, params=self.agent_params)
        self.vehicle_data = self.planner.empty_data_dict()
        self.system_dynamics = Ag._init_system_dynamics(self, params=self.agent_params)

        self.robot_goal = self.robot
        self.commands = None
        return

    def from_conf(self, configs, idx):
        x = float(configs._position_nk2[0][idx][0])
        y = float(configs._position_nk2[0][idx][1])
        th = float(configs._heading_nk1[0][idx][0])
        return (x, y, th)

    def query_next_ckpt(self):
        robot_v_norm = np.sqrt(self.robot_v[0] ** 2 + self.robot_v[1] ** 2)
        robot_w = self.robot_v[2]
        robot_config = SystemConfig.from_pos3(
            self.robot, dt=self.agent_params.dt, v=robot_v_norm, w=robot_w
        )
        planner_data = self.planner.optimize(
            robot_config, self.goal_config, sim_state_hist=self.sim_states
        )
        tmp_goal = Trajectory.new_traj_clip_along_time_axis(
            planner_data["trajectory"],
            self.agent_params.control_horizon,
            repeat_second_to_last_speed=True,
        )
        robot_goal = self.from_conf(tmp_goal, -1)

        return robot_goal

    def joystick_sense(self):
        self.send_to_robot("sense")
        if not self.listen_once():
            self.joystick_on = False

        robot_prev = self.robot.copy()
        agents_prev = {}
        for key in list(self.agents.keys()):
            agent = self.agents[key]
            agents_prev[key] = agent.copy()

        # NOTE: self.sim_dt is available
        self.agents = {}
        self.agents_radius = {}
        agents_info = self.sim_state_now.get_all_agents()
        for key in list(agents_info.keys()):
            agent = agents_info[key]
            self.agents[key] = agent.get_current_config().position_and_heading_nk3(
                squeeze=True
            )
            self.agents_radius[key] = agent.get_radius()
        robot_tmp = list(self.sim_state_now.get_robots().values())[0]
        self.robot = robot_tmp.get_current_config().position_and_heading_nk3(
            squeeze=True
        )
        self.robot_radius = robot_tmp.get_radius()

        self.robot_v = (self.robot - robot_prev) / self.sim_dt
        self.agents_v = {}
        for key in list(self.agents.keys()):
            if key in agents_prev:
                v = (self.agents[key] - agents_prev[key]) / self.sim_dt
            else:
                v = np.array([0, 0, 0], dtype=np.float32)
            self.agents_v[key] = v

        ckpt_radius = 1
        goal_radius = 5
        if euclidean_dist2(self.robot, self.robot_goal) <= ckpt_radius:
            if euclidean_dist2(self.robot, self.robot_goal_final) < goal_radius:
                self.robot_goal = self.robot_goal_final
            else:
                self.robot_goal = self.query_next_ckpt()
        return

    def get_agents(self):
        robot_max_spd = 1.2
        self.env_robot = Agent(
            self.robot[0],
            self.robot[1],
            self.robot_goal[0],
            self.robot_goal[1],
            self.robot_radius + 0.001,
            robot_max_spd,
            self.robot[2],
            CADRLPolicy,
            UnicycleDynamics,
            [OtherAgentsStatesSensor, LaserScanSensor],
            -1,
        )
        env_agents = [self.env_robot]
        for i, key in enumerate(list(self.agents.keys())):
            agent = self.agents[key]
            agent_v = self.agents_v[key]
            agent_goal = self.agents_goals[key]
            agent_radius = self.agents_radius[key]
            env_agent = Agent(
                agent[0],
                agent[1],
                agent_goal[0],
                agent_goal[1],
                agent_radius + 0.001,
                np.sqrt(agent_v[0] ** 2 + agent_v[1] ** 2),
                agent[2],
                NonCooperativePolicy,
                UnicycleDynamics,
                [OtherAgentsStatesSensor, LaserScanSensor],
                i,
            )
            env_agents.append(env_agent)

        return env_agents

    def joystick_plan(self):
        horizon_scale = 10
        horizon = self.sim_dt * horizon_scale
        self.agents_goals = {}
        for key in list(self.agents.keys()):
            goal = self.agents[key] + self.agents_v[key] * horizon
            self.agents_goals[key] = goal

        collision_avoidance_env = CollisionAvoidanceEnv()
        collision_avoidance_env.set_static_map(
            self.map_path, self.map_width, self.map_height, self.map_scale
        )
        agents = self.get_agents()
        [
            agent.policy.initialize_network()
            for agent in agents
            if hasattr(agent.policy, "initialize_network")
        ]
        collision_avoidance_env.set_agents(agents)
        obs = collision_avoidance_env.reset()
        actions = {}
        obs, rewards, game_over, which_agents_done = collision_avoidance_env.step(
            actions, self.sim_dt
        )
        return

    def joystick_act(self):
        pos = self.env_robot.get_agent_data("pos_global_frame")
        x = pos[0]
        y = pos[1]
        print([self.robot, (x, y), self.sim_state_now.sim_t])
        th = np.arctan2(y - self.robot[1], x - self.robot[0])
        if self.joystick_on:
            self.send_cmds([(x, y, th, 0)], send_vel_cmds=False)
        return

    def update_loop(self):
        self.robot_receiver_socket.listen(1)
        self.joystick_on = True
        while self.joystick_on:
            self.joystick_sense()
            self.joystick_plan()
            self.joystick_act()
        self.finish_episode()
        return

