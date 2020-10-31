#pragma once

#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <random>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <sys/time.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <omp.h>

#define SHOW_VELOCITY
// #define SHOW_OBSTACLES
#define SAVE_FIGURES
#define VISUALIZATION
#define LOGGING


class Vector2{
public:
    double x;
    double y;
};



class Robot{
public:
    Vector2 position;
    Vector2 velocity;
    double id;
    double type;
};

class ClusterMetric
{
public:
    ClusterMetric(double robots, double groups, double threshold);
    double robots, groups, threshold;
    int compute(std::vector<Robot> states);
};


class Controller
{
public:
    // WiseRobot Constructor
    Controller(ros::NodeHandle *nodehandle);

    int robots;
    int groups;
    double sensing;
    double worldsize;
    double safezone;
    double mass;
    double vmax;
    double dt;
    double alpha;
    double dAB, dAA;

    int metric_v;
    float max_iteration;

    std::string worldfile;
    std::string logginfile;
    std::ofstream logfile;

    int seed;
    bool gui;
    bool logging;
    std::string swarmconf;

    std::vector<Robot> states;
    std::vector<Vector2> obstacles;

    void update(bool compute_metric);
    bool draw(int step);

private:
    ros::NodeHandle nh_; // ROS Node Handle

    double euclidean(Vector2 a, Vector2 b);
    std::vector<Vector2> getObstaclesPoints(double sensing, Vector2 p);
    bool getIntersection(double sensing, Vector2 circle, Vector2 p1, Vector2 p2, Vector2& o1, Vector2& o2);
    std::vector<std::vector<Robot>> getAllRobotsNeighborns(std::vector<Robot> agents);
    Vector2 saturation(Vector2 v, double norm);
    Vector2 control(Robot r_i, std::vector<Robot> states_t);
};


