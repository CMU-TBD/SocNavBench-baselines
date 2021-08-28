from typing import Any, Dict

from obstacles.obstacle_map import ObstacleMap

from joystick_py.joystick_remote import JoystickRemote
from utils.utils import color_text


class JoystickSocialForce(JoystickRemote):
    def __init__(self):
        print(
            f"{color_text['orange']}",
            "NOTE: in order to run the social-forces executable\n",
            "      first go to the joystick/social_force directory\n",
            "      and run `export LD_LIBRARY_PATH=./src`",
            f"{color_text['reset']}",
        )
        super().__init__(algo_name="social_force", HOST="127.0.0.1", PORT=2112)

    def init_obstacle_map(self, env: Dict[str, Any]) -> ObstacleMap:
        scale = float(env["map_scale"])
        map_trav = env["map_traversible"]
        self.map_height = len(map_trav) * scale
        self.map_width = len(map_trav[0]) * scale
        return super().init_obstacle_map(env)

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
            + str(self.robot_radius + 0.001)
            + "\n"
        )
        info_string += "agents\n"
        for key in list(self.agents.keys()):
            agent = self.agents[key]
            agent_v = self.agents_v[key]
            agent_goal = self.agents_goals[key]
            agent_radius = self.agents_radius[key]
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
                + str(agent_radius + 0.001)
                + "\n"
            )
        info_string += "map_size\n"
        info_string += str(self.map_height) + "," + str(self.map_width) + "\n"
        info_string += "End*"
        return info_string
