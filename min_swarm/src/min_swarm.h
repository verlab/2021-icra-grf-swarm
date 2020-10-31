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
    double theta;
};

class Robot{
public:
	Vector2 position;
	Vector2 velocity;
    Vector2 acceleration;
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
    double vmax;
    double dt;
    double r;

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
    boost::mutex mutex;

	void update();
    bool draw(int step);


private:
    ros::NodeHandle nh_; // ROS Node Handle
    double warp(double theta);
    double deg2rad(double deg);
    Vector2 fwd(double x, double y, double theta, double omega, double R, double dt);
    Vector2 fwd_0(Vector2 p, double V, double l, double dt);
    Vector2 fwd_1(Vector2 p, double V, double l, double dt);
    Vector2 fwd_2(Vector2 p, double V, double l, double dt);
    bool sense(Vector2 p1, Vector2 p2, double r);
    bool sense2(Vector2 p1, Vector2 p2, double half_beam_angle);
    double euclidean(Vector2 a, Vector2 b);
    Vector2 saturation(Vector2 v, double norm);
    void control(Robot r_i, std::vector<Robot> states_t);
};


