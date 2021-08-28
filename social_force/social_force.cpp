#include <ped_includes.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream> 

#include <unistd.h> 
#include <stdio.h> 
#include <math.h>
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 

#define MAX_AGENT 500
#define MAX_OBSTACLE 9999
#define PORT 2112

#define SF_FACTOR 10
#define DE_FACTOR 1
#define OB_FACTOR 3

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
    float map_height;
    float map_width;
    struct Robot robot;
    struct Agent agents[MAX_AGENT];
    struct Obstacle obstacles[MAX_OBSTACLE];
    float delta_t;
};

struct Info information;

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
            if (element.compare("map_size") == 0) {
                mode = 5;
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
            if (mode == 5) {
                information.map_height = std::stof(elem_list[0]);
                information.map_width = std::stof(elem_list[1]);
            }
        } else {
            skip = false;
        }
        
        elem_list.clear();
    }
    return;
}

void receiveInformation(int socket) {
    std::string info_str;
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

void sendCommands(Ped::Tvector robot_pos, int socket) {
    std::string command = std::to_string(robot_pos.x);
    command += ",";
    command += std::to_string(robot_pos.y);
    std::cout << "=========================" << std::endl;
    const char* command_c = command.c_str();
    send(socket, command_c, strlen(command_c), 0);
    return;
}

int main()
{
    int socket;
    int end_flag;
    float goal_radius = 0.001;
    socket = establishConnection();

    while (true) {
        receiveInformation(socket);

        Ped::Tscene *pedscene = new Ped::Tscene(0, 0, information.map_width, information.map_height);

        Ped::Tagent *robot = new Ped::Tagent();
        Ped::Twaypoint *robot_goal = new Ped::Twaypoint(
            information.robot.goal_x, information.robot.goal_y, goal_radius);
        robot->addWaypoint(robot_goal);

        float crit_dist = 4 * information.robot.radius;
        float goal_dist = sqrt(pow(information.robot.x - information.robot.goal_x, 2) + 
                               pow(information.robot.y - information.robot.goal_y, 2));
        if (goal_dist <= crit_dist) {
            robot->setfactorsocialforce(SF_FACTOR);
            robot->setfactordesiredforce(1000);
            robot->setfactorobstacleforce(OB_FACTOR);
        } else {
            robot->setfactorsocialforce(SF_FACTOR);
            robot->setfactordesiredforce(DE_FACTOR);
            robot->setfactorobstacleforce(OB_FACTOR);
        }

        robot->setPosition(information.robot.x, information.robot.y, 0);
        robot->setVelocity(information.robot.vx, information.robot.vy, 0);
        robot->setVmax(1.2);
        pedscene->addAgent(robot);

        for (int i = 0; i < information.num_agents; i++) {
            Ped::Tagent *a = new Ped::Tagent();
            Ped::Twaypoint *g = new Ped::Twaypoint(
                information.agents[i].goal_x, information.agents[i].goal_y, goal_radius);
            a->addWaypoint(g);
            a->setPosition(information.agents[i].x, information.agents[i].y, 0);
            a->setVelocity(information.agents[i].vx, information.agents[i].vy, 0);
            a->setVmax(sqrt(pow(information.agents[i].vx, 2) + pow(information.agents[i].vy, 2)));
            pedscene->addAgent(a);
        }

        for (int i = 0; i < information.num_obstacles; i++) {
            Ped::Tobstacle *o = new Ped::Tobstacle(
                information.obstacles[i].x1, information.obstacles[i].y1,
                information.obstacles[i].x2, information.obstacles[i].y2);
            pedscene->addObstacle(o);
        }

        Ped::Tvector robot_old_pos = robot->getPosition();
        
        pedscene->moveAgents(information.delta_t);

        Ped::Tvector robot_new_pos = robot->getPosition();
        Ped::Tvector robot_vel = robot_new_pos - robot_old_pos;
        std::cout << robot_vel.x << ", " << robot_vel.y << std::endl;
        std::cout << robot_new_pos.x << ", " << robot_new_pos.y << std::endl;

        sendCommands(robot_new_pos, socket);

        for (Ped::Twaypoint *g : pedscene->getAllWaypoints()) delete g;
        for (Ped::Tagent *a : pedscene->getAllAgents()) delete a;
        for (Ped::Tobstacle *o : pedscene->getAllObstacles()) delete o;
        delete pedscene;
    }
            
    return 0;
}
