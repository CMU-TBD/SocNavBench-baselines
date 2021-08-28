from joystick_py.joystick_remote import JoystickRemote


class JoystickRVO(JoystickRemote):
    def __init__(self):
        super().__init__(algo_name="RVO", HOST="127.0.0.1", PORT=2111)

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
