#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <ctime>
#include <sys/time.h>
#include "ros/ros.h"
#include "orca.h"
#include <stdlib.h>
#include <time.h>

using namespace cv;
using namespace std;

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

#define SAVE_FIGURE
#define LOGGING
#define VISUALIZATION

int metric_v = 0;

class mVector2{
public:
    double x;
    double y;
};

class Robot{
public:
    mVector2 position;
    mVector2 velocity;
    double id;
    double type;
};

class ClusterMetric
{
public:
    ClusterMetric(double robots_, double groups_, double threshold_) : robots (robots_), groups (groups_), threshold(threshold_){}
    double robots, groups, threshold;

    int compute(std::vector<Robot> states){
        std::vector<std::vector<Robot>> clusters;

        while (states.size()){
            std::vector<Robot> cluster;    
            cluster.push_back(states[0]);
            states.erase(states.begin());
            bool founded = true;
            while (founded){
                founded = false;
                for (int i = 0; i < cluster.size(); i++){
                    for (int j = 0; j < states.size(); j++){
                        if (cluster[i].type != states[j].type){
                            continue;
                        }
                        double dx = cluster[i].position.x - states[j].position.x;
                        double dy = cluster[i].position.y - states[j].position.y;
                        double dist = sqrt(dx * dx + dy * dy);
                        if (dist > this->threshold){
                            continue;
                        }
                        cluster.push_back(states[j]);
                        states.erase(states.begin()+j);
                        founded = true;
                        break;
                    }
                }
            }
            clusters.push_back(cluster);
        }

        return (int) clusters.size();
    }
};

void saturation(double &vx, double &vy, double norm){
    double r = fabs(vy/vx);
    if ((r >= 1.0) && (fabs(vy) > norm)){
        vy = (vy * norm)/fabs(vy); 
        vx = (vx * norm)/(r * fabs(vx));
    } else if ((r < 1.0) && (fabs(vx) > norm)){
        vx = (vx * norm)/fabs(vx); 
        vy = (vy * norm * r)/(fabs(vy)); 
    } else {
        vx = vx;
        vy = vy;
    }
}



void updateVisualization(ORCA_Simulator *sim){
    /* Output the current global time. */
    std::cout << sim->getGlobalTime();

    /* Output the current position of all the agents. */
    for (int i = 0; i < sim->getNumAgents(); ++i) {
        std::cout << " " << sim->getAgentPosition(i);
    }

    std::cout << std::endl;
}

void setPreferredVelocities(ORCA_Simulator *sim){
    
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
    
        Vector2 goalVector = sim->getAgentGoal(i) - sim->getAgentPosition(i);
        if (absSq(goalVector) > 1.0f) {
            goalVector = normalize(goalVector);
        }

        sim->setAgentPrefVelocity(i, goalVector);
    }
}

bool reachedGoal(ORCA_Simulator *sim, int numAgents, int flockSize){
    
    //int nFlocks = numAgents/flockSize;
    //float avg[nFlocks];

    /*for(int i = 0; i<nFlocks; i++){
        avg[i] = 0.0f;
    }

    for (int i = 0; i < numAgents; i++){
        if(abs(sim->agents_[i]->position_-sim->agents_[i]->goal_) > 3){
            return false;
        }
    }
    */
    //for (int i = 0; i < numAgents; i++){
    //    if(sim->agents_[i]->numNeighbors_ < numAgents - 1){
    //        return false;
    //    }
    //}

    if(sim->mst() == 0){
        return false;
    }else{
        return true;
    }
}


bool plot(ORCA_Simulator *sim, int numAgents, int flockSize, int iterations, Mat copy_image){
    Scalar color, colorLine;
    cv::rectangle( copy_image, cv::Point(50,50), cv::Point(650,650), cv::Scalar(0,0,0), 4, 8);

    cv::putText(copy_image, //target image
            std::to_string(iterations), //text
            cv::Point(copy_image.cols/2, 35), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            0.6,
            CV_RGB(0, 0, 0), //font color
            1);

        float c = 300.0/5.0;
    cv::putText(copy_image, "Metric: " + std::to_string(metric_v), cv::Point(50, 35), cv::FONT_HERSHEY_DUPLEX,
            0.6, CV_RGB(0, 0, 0), 1);
        
    for(int i = 0; i < numAgents; i++){
        switch((int)sim->agents_[i]->flock_id_){
            case 0: color = cv::Scalar(128,0,0); break;         // maroon   
            case 1: color = cv::Scalar(47,79,79); break;       // dark slate gray  
            case 2: color = cv::Scalar(138,43,226); break;     // blue violet  
            case 3: color = cv::Scalar(199,21,133); break;     // medium violet red    
            case 4: color = cv::Scalar(144,238,144); break;    // light green  
            case 5: color = cv::Scalar(255,215,0); break;       // gold 
            case 6: color = cv::Scalar(218,165,32); break;      // golden rod   
            case 7: color = cv::Scalar(189,183,107); break;     // dark khaki   
            case 8: color = cv::Scalar(128,128,0); break;       // olive    
            case 9: color = cv::Scalar(154,205,50); break;      // yellow green 
            case 10: color = cv::Scalar(107,142,35); break;     // olive drab   
            case 11: color = cv::Scalar(127,255,0); break;      // chart reuse  
            case 12: color = cv::Scalar(0,100,0); break;        // dark green   
            case 13: color = cv::Scalar(255,140,0); break;       // dark orange  
            case 14: color = cv::Scalar(46,139,87); break;      // sea green    
            case 15: color = cv::Scalar(102,205,170); break;    // medium aqua marine   
            case 16: color = cv::Scalar(220,20,60); break;       // crimson  
            case 17: color = cv::Scalar(0,139,139); break;      // dark cyan
            case 18: color = cv::Scalar(0,255,255); break;      // cyan 
            case 19: color = cv::Scalar(70,130,180); break;     // steel blue   
            case 20: color = cv::Scalar(100,149,237); break;    // corn flower blue 
            case 21: color = cv::Scalar(30,144,255); break;     // dodger blue  
            case 22: color = cv::Scalar(0,0,128); break;        // navy 
            case 23: color = cv::Scalar(240,128,128); break;     // light coral  
            case 24: color = cv::Scalar(75,0,130); break;       // indigo   
            case 25: color = cv::Scalar(139,0,139); break;      // dark magenta 
            case 26: color = cv::Scalar(238,130,238); break;    // violet   
            case 27: color = cv::Scalar(255,160,122); break;     // light salmon 
            case 28: color = cv::Scalar(255,105,180); break;    // hot pink 
            case 29: color = cv::Scalar(112,128,144); break;    // slate gray   
            default: color = cv::Scalar(0,0,0); break;          // black  
        }
        std::swap(color[0], color[2]);


        Point vel;
        double vx = sim->agents_[i]->velocity_.x();
        double vy = sim->agents_[i]->velocity_.y();
        saturation(vx, vy, 0.3);
        int x = 350 + c * (sim->agents_[i]->position_.x() + vx);
        int y = 350 - c * (sim->agents_[i]->position_.y() + vy);
        cv::arrowedLine( copy_image, cv::Point(350 + c * sim->agents_[i]->position_.x(),
               350 - c * sim->agents_[i]->position_.y()), cv::Point(x, y), cv::Scalar( 220, 220, 220), 2, 8);

        circle(copy_image, Point(350 + c * sim->agents_[i]->position_.x(),
               350 - c * sim->agents_[i]->position_.y() ), c * 0.07 * sim->agents_[i]->radius_/sim->agents_[i]->radius_, color, -1, 8);

        // //raio de visao
        // circle(copy_image, Point(350 + c*sim->agents_[i]->position_.x(),
        //       350 - c*sim->agents_[i]->position_.y() ), c*sim->agents_[i]->neighborDist_, color, 1, 8);

    }
    char filenanme[17];
    #ifdef SAVE_FIGURE
        std::sprintf (filenanme, "image_%06d.png", iterations);
        cv::imwrite(filenanme, copy_image);
    #endif

    return (cv::waitKey(1) != 27);
}

// void setupScenario(ORCA_Simulator *sim, int numAgents, int flockSize, const char * file_name)
// {
//     FILE * agents_coord = fopen(file_name, "r");
//     float  x, y, gx, gy;
//     srand( (unsigned)time(NULL) );

//     fscanf(agents_coord, "%f", &x);
//     fscanf(agents_coord, "%f", &x);



//     printf("Agentes %d\n", numAgents);
//     for (int i = 0; i < numAgents; ++i) {
        
//         //Posicoes definidas por arquivo
//         fscanf(agents_coord, "%f", &x);
//         fscanf(agents_coord, "%f", &y);
        
//         //Posicoes alvo definidas por arquivo
//         fscanf(agents_coord, "%f", &gx);
//         fscanf(agents_coord, "%f", &gy);
                


//        /**/
//         sim->addAgent(x, y, static_cast<int>(numAgents/flockSize));
//         sim->agents_[i]->goal_ = Vector2(gx, gy);
//         double vx = (double)gx, vy = (double)gy;
//         saturation(vx, vy, 1.0);
//         sim->setAgentVelocity(i, Vector2(vx, vy));
//         sim->setAgentPrefVelocity(i, Vector2(vx, vy));

//         //Flocks de mesmo tamanho
//         //sim->agents_[i]->setFlockID(static_cast <int> (i/flockSize));
//         sim->agents_[i]->setFlockID(static_cast <int>(i%(numAgents/flockSize)));
        

//         //Flocks definidos pelo arquivo
//         fscanf(agents_coord, "%f", &gy);
//         sim->agents_[i]->setFlockID(gy);
        
//         sim->agents_[i]->heading_ = atan2(sim->agents_[i]->goal_.y()-sim->agents_[i]->position_.y(), sim->agents_[i]->goal_.x()-sim->agents_[i]->position_.x())*(180/M_PI);
    
//         //fprintf(log_simulation, "%f;%f;%f;%f;%d\n", x, y, gx, gy, sim->agents_[i]->flock_id_);

//     }
//     fclose(agents_coord);
//     //fclose(log_simulation);
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "pso_controller", ros::init_options::AnonymousName);

    /* Get params */
    int numAgents, flockSize, nFlocks, seed = 1;
    bool gui = false;
    ros::param::get("~nrobots", numAgents);
    ros::param::get("~ngroups", nFlocks);
    ros::param::get("~seed", seed);
    ros::param::get("~gui", gui);
    flockSize = numAgents/nFlocks;

    float sensing_range = 0.5;
    ros::param::get("~sensing", sensing_range);

    int worldsize = 5;
    ros::param::get("~worldsize", worldsize);    

    float max_iteration = 20000;
    ros::param::get("~iterations", max_iteration);
    // std::cout << max_iteration << std::endl;

    std::string swarmconf;
    ros::param::get("~swarmconf", swarmconf);
    // std::cout << swarmconf << std::endl;

    std::string worldfile, logginfile;
    bool logging = false;
    ros::param::get("~log", logging);
    if (logging){
        logginfile = "pso_swarm_r"+std::to_string(numAgents)+"_g_"+std::to_string(nFlocks)+"_s_"+std::to_string(sensing_range)+"_w_"+std::to_string(worldsize)+"_run_"+std::to_string(seed)+".log";
    }
    std::cout << logginfile << std::endl;

    ORCA_Simulator *sim = new ORCA_Simulator();
    sim->setTimeStep(0.02f);
    //float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed
    sim->setAgentDefaults(sensing_range, 10, 3.0f, 3.0f, 0.09f, 1.0f);
    /* Load swarm initial configuration */
    std::istringstream ss(swarmconf);
    for (int i = 0; i < numAgents; ++i) {
        float x, y, gx, gy, id;
        ss >> x; ss >> y; ss >> gx; ss >> gy; ss >> id;
        
        sim->addAgent(x, y, static_cast<int>(nFlocks));
        sim->agents_[i]->goal_ = Vector2(gx, gy);
        double vx = (double)gx, vy = (double)gy;
        saturation(vx, vy, 1.0);
        sim->setAgentVelocity(i, Vector2(vx, vy));
        sim->setAgentPrefVelocity(i, Vector2(vx, vy));

        sim->agents_[i]->setFlockID(static_cast <int>(id));
        
        sim->agents_[i]->heading_ = atan2(sim->agents_[i]->goal_.y()-sim->agents_[i]->position_.y(), sim->agents_[i]->goal_.x()-sim->agents_[i]->position_.x())*(180/M_PI);
    }
    int vec[numAgents];

    for(int i = 0; i < numAgents; i++){
            vec[i] = 0;
    }

    cv::Mat image(700, 700, CV_8UC3, cv::Scalar(255,255,255));    
    Mat copy_image;
    if (gui){
        namedWindow("PSO-Segregation (Inácio et al. 2019)", WINDOW_AUTOSIZE);
    }

    printf("\n%s\n", "Simulation started.");

    int j = 0;
    
    ofstream logfile;
    if (logging){
        logfile.open (logginfile);
        ROS_INFO("\33[92mLog: %s\33[0m", logginfile.c_str());
    }
   
    bool runnig = true;
    ClusterMetric metric(numAgents, nFlocks, 0.3);


    float iteration = 0.0f;
    do{
        std::vector<Robot> states;
        for(int i = 0; i < numAgents; i++){
            Robot ri;
            ri.position.x = sim->agents_[i]->position_.x();
            ri.position.y = sim->agents_[i]->position_.y();
            ri.type = sim->agents_[i]->flock_id_;
            states.push_back(ri);
        }
        metric_v = metric.compute(states);
        if (logging){
            logfile << metric_v << "\n" ;
        }
                           
        setPreferredVelocities(sim);
        sim->PSODoStep(numAgents, nFlocks);
        sim->orcaDoStep();
        if (gui){
            copy_image = image.clone(); 
            runnig = plot(sim, numAgents, flockSize, iteration, copy_image);
            imshow("PSO-Segregation (Inácio et al. 2019)", copy_image);    
            waitKey(1);            
        }

        iteration = iteration + 1.0f;
    
    } while (ros::ok() && runnig && (iteration < max_iteration));

    if (logfile){
        logfile.close();
    }
    printf("Iterations: %f\n", iteration);
    if (gui){
        waitKey(0);
    }
    
    sim->~ORCA_Simulator();
    sim = NULL;

    printf("Simulation finished.\n");

    return EXIT_SUCCESS;
}
