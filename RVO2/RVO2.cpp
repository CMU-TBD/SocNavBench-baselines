#include <RVO.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream> 

#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 

#define MAX_AGENT 500
#define MAX_OBSTACLE 9999
#define PORT 2111

struct Agent
{
    std::string name;
    float x;
    float y;
    float th;
    float vx;
    float vy;
    float vth;
    float goal_x;
    float goal_y;
    float goal_th;
    float radius;
};

struct Robot
{
    float x;
    float y;
    float th;
    float vx;
    float vy;
    float vth;
    float goal_x;
    float goal_y;
    float goal_th;
    float radius;
};

struct Obstacle
{
    float x1;
    float x2;
    float y1;
    float y2;
};

struct Info
{
    int num_agents;
    int num_obstacles;
    struct Robot robot;
    struct Agent agents[MAX_AGENT];
    struct Obstacle obstacles[MAX_OBSTACLE];
    float delta_t;
};

struct Info information;
RVO::Vector2 robot_goal;

int establishConnection() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        std::cout << "socket failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        std::cout << "setsockopt failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );

    if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        std::cout << "bind_failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        std::cout << "listen failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address, (socklen_t*) &addrlen)) < 0)
    {
        std::cout << "accept failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    std::cout << "Waiting for signal..." << std::endl;

    return new_socket;
}

void parseInformation(std::string info_string) {
    std::istringstream f(info_string);
    std::string line;
    std::string element;
    std::vector<std::string> elem_list;
    int mode = 0;
    bool skip = false;
    information.num_obstacles = 0;
    information.num_agents = 0;
    while (std::getline(f, line)) {
        std::stringstream line_tmp(line);
        while (std::getline(line_tmp, element, ',')) {
            elem_list.push_back(element);
        }
        if (elem_list.size() == 1) {
            element = elem_list[0];
            if (element.compare("delta_t") == 0) {
                mode = 1;
                skip = true;
            }
            if (element.compare("obstacles") == 0) {
                mode = 2;
                skip = true;
            }
            if (element.compare("robot") == 0) {
                mode = 3;
                skip = true;
            }
            if (element.compare("agents") == 0) {
                mode = 4;
                skip = true;
            }
            if (element.compare("End*") == 0) {
                mode = 0;
                skip = true;
            }
        }

        if (skip == false) {
            if (mode == 1) {
                element = elem_list[0];
                information.delta_t = std::stof(element);
            }
            if (mode == 2) {
                information.obstacles[information.num_obstacles].x1 = std::stof(elem_list[0]);
                information.obstacles[information.num_obstacles].y1 = std::stof(elem_list[1]);
                information.obstacles[information.num_obstacles].x2 = std::stof(elem_list[2]);
                information.obstacles[information.num_obstacles].y2 = std::stof(elem_list[3]);
                information.num_obstacles++;
            }
            if (mode == 3) {
                information.robot.x = std::stof(elem_list[0]);
                information.robot.y = std::stof(elem_list[1]);
                information.robot.th = std::stof(elem_list[2]);
                information.robot.vx = std::stof(elem_list[3]);
                information.robot.vy = std::stof(elem_list[4]);
                information.robot.vth = std::stof(elem_list[5]);
                information.robot.goal_x = std::stof(elem_list[6]);
                information.robot.goal_y = std::stof(elem_list[7]);
                information.robot.goal_th = std::stof(elem_list[8]);
                information.robot.radius = std::stof(elem_list[9]);
            }
            if (mode == 4) {
                information.agents[information.num_agents].name = elem_list[0];
                information.agents[information.num_agents].x = std::stof(elem_list[1]);
                information.agents[information.num_agents].y = std::stof(elem_list[2]);
                information.agents[information.num_agents].th = std::stof(elem_list[3]);
                information.agents[information.num_agents].vx = std::stof(elem_list[4]);
                information.agents[information.num_agents].vy = std::stof(elem_list[5]);
                information.agents[information.num_agents].vth = std::stof(elem_list[6]);
                information.agents[information.num_agents].goal_x = std::stof(elem_list[7]);
                information.agents[information.num_agents].goal_y = std::stof(elem_list[8]);
                information.agents[information.num_agents].goal_th = std::stof(elem_list[9]);
                information.agents[information.num_agents].radius = std::stof(elem_list[10]);
                information.num_agents++;
            }
        } else {
            skip = false;
        }
        
        elem_list.clear();
    }
    return;
}

void receiveInformation(int socket) {
    std::string info_str = "";
    char buffer[1024];
    int byte_count;
    do {
        byte_count = read(socket, buffer, 1023);
        buffer[byte_count] = '\0';
        info_str += buffer;
    } while (buffer[byte_count - 1] != '*');
    //std::cout << info_str << std::endl;
    parseInformation(info_str);
    return;
    
}

void sendCommands(RVO::Vector2 robot_pos, int socket) {
    std::string command = std::to_string(robot_pos.x());
    command += ",";
    command += std::to_string(robot_pos.y());
    const char* command_c = command.c_str();
    send(socket, command_c, strlen(command_c), 0);
    return;
}

void setupScenario(RVO::RVOSimulator* sim) {
    sim->setTimeStep(information.delta_t);
    
    sim->setAgentDefaults(50.0f, 50, 5.0f, 5.0f, information.robot.radius, 1.2f);

    sim->addAgent(RVO::Vector2(information.robot.x, information.robot.y));
    for (int i = 0; i < information.num_agents; i++) {
        sim->addAgent(RVO::Vector2(information.agents[i].x, information.agents[i].y));
    }


    //sim->setAgentMaxSpeed(0, 1.2f);
    RVO::Vector2 v(information.robot.vx, information.robot.vy);
    sim->setAgentVelocity(0, v);
    for (int i = 1; i < sim->getNumAgents(); i++) {
        sim->setAgentRadius(i, information.agents[i - 1].radius);
        sim->setAgentNeighborDist(i, 0.0);
        sim->setAgentMaxNeighbors(i, 0);
        //sim->setAgentTimeHorizon(i, 0.01f);
        //sim->setAgentTimeHorizonObst(i, 0.01f);
        RVO::Vector2 v(information.agents[i - 1].vx, information.agents[i - 1].vy);
        sim->setAgentVelocity(i, v);
        sim->setAgentMaxSpeed(i, abs(v));
    }

    robot_goal = RVO::Vector2(information.robot.goal_x, information.robot.goal_y);

    for (int i = 0; i < information.num_obstacles; i++) {
        std::vector<RVO::Vector2> vertices;
        vertices.push_back(RVO::Vector2(information.obstacles[i].x1, information.obstacles[i].y1));
        vertices.push_back(RVO::Vector2(information.obstacles[i].x2, information.obstacles[i].y2));
        sim->addObstacle(vertices);
        vertices.clear();
    }

    sim->processObstacles();
    return;
}

void setPreferredVelocities(RVO::RVOSimulator* sim) {
    sim->setAgentPrefVelocity(0, normalize(robot_goal - sim->getAgentPosition(0)));
    for (int i = 1; i < sim->getNumAgents(); i++) {
        sim->setAgentPrefVelocity(i, normalize(
            RVO::Vector2(information.agents[i - 1].vx, information.agents[i - 1].vy)));
    }
    return;
}

void updateVisualization(RVO::RVOSimulator* sim, int socket) {
    RVO::Vector2 robot_prev_pos(information.robot.x, information.robot.y);
    RVO::Vector2 robot_pos = sim->getAgentPosition(0);
    RVO::Vector2 robot_vel = sim->getAgentVelocity(0);

    /*
    float dist;
    int min_idx = 0;
    float min_dist = 1000;
    for (int i = 1; i < sim->getNumAgents(); i++) {
        RVO::Vector2 a_pos = sim->getAgentPosition(i);
        dist = abs(a_pos - robot_pos);
        if (dist < min_dist) {
            min_idx = i;
            min_dist = dist;
        }
    }

    if ((min_idx > 0) && 
       (min_dist <= standard)) {
        RVO::Vector2 agent_origin(information.agents[min_idx-1].x, information.agents[min_idx-1].y);
        RVO::Vector2 agent_pos = sim->getAgentPosition(min_idx);
        RVO::Vector2 agent_est_pos(
        information.agents[min_idx-1].x + information.agents[min_idx-1].vx * information.delta_t,
        information.agents[min_idx-1].y + information.agents[min_idx-1].vy * information.delta_t);

        if ((abs(agent_est_pos - agent_origin) > 0.0001) && 
            (abs(agent_pos - agent_origin) > 0.0001) &&
            (abs(robot_pos - robot_prev_pos) > 0.0001)) {
            RVO::Vector2 diff = normalize(agent_est_pos - agent_origin) - 
                                normalize(agent_pos - agent_origin);
            robot_pos = (normalize(robot_pos - robot_prev_pos) + diff) 
                        * abs(robot_pos - robot_prev_pos) + robot_prev_pos;
        }
    }
    */

    //std::cout << robot_prev_pos  << std::endl;
    std::cout << robot_pos << std::endl;
    //std::cout << robot_pos - robot_prev_pos  << std::endl;
    std::cout << robot_vel << std::endl;
    //std::cout << abs(robot_vel)  << ", " << abs(robot_pos - robot_prev_pos)  << std::endl;
    std::cout << "==========================================" << std::endl;
    sendCommands(robot_pos, socket);
    return;
}

int main()
{
    int socket;
    socket = establishConnection();

    while (true) {
        receiveInformation(socket);

        RVO::RVOSimulator* sim = new RVO::RVOSimulator();

        setupScenario(sim);

        setPreferredVelocities(sim);
        sim -> doStep();
        updateVisualization(sim, socket);
        
        delete sim;
    }

    return 0;
}

