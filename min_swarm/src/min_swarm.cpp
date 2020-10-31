#include "min_swarm.h"

ClusterMetric::ClusterMetric(double robots_, double groups_, double threshold_) : robots (robots_), groups (groups_), threshold(threshold_){}

int ClusterMetric::compute(std::vector<Robot> states){
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

Controller::Controller(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    /* Get params */
    int flockSize;
    this->seed = 1;
    this->gui = false;
    ros::param::get("~nrobots", this->robots);
    ros::param::get("~ngroups", this->groups);
    ros::param::get("~seed", this->seed);
    ros::param::get("~gui", this->gui);
    flockSize = this->robots/this->groups;

    this->sensing = 0.5;
    this->safezone = 0.3;
    ros::param::get("~sensing", this->sensing);

    this->worldsize = 5.0;
    ros::param::get("~worldsize", this->worldsize);    

    this->max_iteration = 20000;
    ros::param::get("~iterations", this->max_iteration);
    // std::cout << max_iteration << std::endl;

    
    ros::param::get("~swarmconf", this->swarmconf);
    // std::cout << swarmconf << std::endl;

    this->logging = false;
    ros::param::get("~log", this->logging);
    if (this->logging){
        this->logginfile = "min_swarm_r"+std::to_string(this->robots)+"_g_"+std::to_string(this->groups)+"_s_"+std::to_string(this->sensing)+"_w_"+std::to_string(this->worldsize)+"_run_"+std::to_string(this->seed)+".log";
    }
    std::cout << this->logginfile << std::endl;
    
    this->vmax = 1.0;
    this->dt = 0.02;
    this->r = 0.070;
    
    ROS_INFO("%d %d", this->robots, this->groups);

    if (this->logging){
        this->logfile.open(this->logginfile);
        ROS_INFO("\33[92mLog: %s\33[0m", logginfile.c_str());
    }
    std::istringstream ss(this->swarmconf);
    for (int i = 0; i < this->robots; ++i){
        Robot r;
        float x, y, vx, vy, type;
        ss >> x; ss >> y; ss >> vx; ss >> vy; ss >> type;
        // Get initial positions
        r.position.x = x; r.position.y = y;
        
        // Get initial velocities
        r.velocity.x = vx; r.velocity.y = vy;

        r.type = type;
        r.id = i;
        this->states.push_back(r);
    }
    if (this->gui){
        cv::namedWindow("Minimalist-Segregation (Mitrano et al. 2019)", cv::WINDOW_AUTOSIZE);
    }
}


bool Controller::draw(int step){
    // Create board
    cv::Mat board(700, 700, CV_8UC3, cv::Scalar(255,255,255));
    cv::Scalar color;
    cv::rectangle( board, cv::Point(50,50), cv::Point(650,650), cv::Scalar(0,0,0), 4, 8);

    cv::putText(board, std::to_string(step), cv::Point(board.cols/2, 35), cv::FONT_HERSHEY_DUPLEX,
            0.6, CV_RGB(0, 0, 0), 1);

    cv::putText(board, "Metric: " + std::to_string(this->metric_v), cv::Point(50, 35), cv::FONT_HERSHEY_DUPLEX,
            0.6, CV_RGB(0, 0, 0), 1);

    float c = 300.0/5.0;

    #ifdef SHOW_VELOCITY
        for(int i = 0; i < this->robots; i++){
            Vector2 vel;
            vel = this->saturation(this->states[i].velocity, 0.3);
            vel.x = 350 + c * (this->states[i].position.x + vel.x);
            vel.y = 350 - c * (this->states[i].position.y + vel.y);
            cv::arrowedLine( board, cv::Point(350 + c * this->states[i].position.x,
                   350 - c*this->states[i].position.y), cv::Point(vel.x, vel.y), cv::Scalar( 220, 220, 220), 2, 8);
        }
    #endif

    for(int i = 0; i < this->robots; i++){
        switch((int)this->states[i].type){ // BGR
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
        cv::circle(board, cv::Point(350 + c * this->states[i].position.x,
               350 - c*this->states[i].position.y), c*0.07, color, -1, 8);

        // cv::circle(board, cv::Point(350 + c * this->states[i].position.x,
        //        350 - c*this->states[i].position.y), c*this->sensing, color, 1, 8);
    }


    // #ifdef SHOW_OBSTACLES
    //     for(int i = 0; i < this->obstacles.size(); i++){
    //         cv::circle(board, cv::Point(350 + c * this->obstacles[i].x,
    //                350 - c*this->obstacles[i].y), c*0.05, cv::Scalar( 0, 0, 255), -1, 8);
    //     }
    //     this->obstacles.clear();
    // #endif

    cv::imshow("Minimalist-Segregation (Mitrano et al. 2019)", board);
    #ifdef SAVE_FIGURES
        char filenanme[17];
        std::sprintf (filenanme, "image_%06d.png", step);
        cv::imwrite(filenanme, board);
    #endif
    return (cv::waitKey(1) != 27);
}

double Controller::deg2rad(double deg){
    return deg * M_PI/180.0;
}

double Controller::warp(double theta){
    return std::fmod(theta + M_PI, 2 * M_PI) - M_PI;
}

Vector2 Controller::fwd(double x, double y, double theta, double omega, double R, double dt){
    double dtheta = omega * dt;
    double x_c = x + cos(theta + M_PI/2.0) * R;
    double y_c = y + sin(theta + M_PI/2.0) * R;
    Vector2 p_;
    p_.x = x_c + cos(dtheta) * (x - x_c) - sin(dtheta) * (y - y_c);
    p_.y = y_c + sin(dtheta) * (x - x_c) + cos(dtheta) * (y - y_c);
    p_.theta = this->warp(theta + dtheta);
    return p_;
}

Vector2 Controller::fwd_0(Vector2 p, double V, double l, double dt){
    double omega = (4.0 * V)/(3.0 * l);
    double R = -l/4.0;
    return this->fwd(p.x, p.y, p.theta, omega, R, dt);
}

Vector2 Controller::fwd_1(Vector2 p, double V, double l, double dt){
    double omega = (2.0 * V)/(3.0 * l);
    double R = l;
    return this->fwd(p.x, p.y, p.theta, omega, R, dt);
}

Vector2 Controller::fwd_2(Vector2 p, double V, double l, double dt){
    double omega = (2.0 * V)/(l);
    double R = 0.0;
    return this->fwd(p.x, p.y, p.theta, omega, R, dt);
}

bool Controller::sense(Vector2 p1, Vector2 p2, double r){
    double dy = p2.y - p1.y;
    double dx = p2.x - p1.x;
    double x = sqrt(dx * dx + dy * dy);
    double phi = atan2(dy, dx);
    double dphi = atan2(r, x);
    double phi_lower = phi - dphi;
    double phi_upper = phi + dphi;

    if (p1.theta <= phi_upper && p1.theta >= phi_lower){
        return true;
    } else{
        return false;
    }
}

bool Controller::sense2(Vector2 p1, Vector2 p2, double half_beam_angle){
    double dy = p2.y - p1.y;
    double dx = p2.x - p1.x;
    double phi = atan2(dy, dx) - p1.theta;

    if ((phi <= half_beam_angle) && (phi >= -half_beam_angle)){
        return true;
    } else {
        return false;
    }
}

double Controller::euclidean(Vector2 a, Vector2 b){
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy) + 1.0e-9;
}


Vector2 Controller::saturation(Vector2 v, double norm){
    Vector2 vnorm;
    double r = fabs(v.y/v.x);
    if ((r >= 1.0) && (fabs(v.y) > norm)){
        vnorm.y = (v.y * norm)/fabs(v.y); 
        vnorm.x = (v.x * norm)/(r * fabs(v.x));
    } else if ((r < 1.0) && (fabs(v.x) > norm)){
        vnorm.x = (v.x * norm)/fabs(v.x); 
        vnorm.y = (v.y * norm * r)/(fabs(v.y)); 
    } else {
        vnorm.x = v.x;
        vnorm.y = v.y;
    }
    return vnorm;
}


void Controller::control(Robot r_i, std::vector<Robot> states_t){
    /* Get the closest neighborn return by the sensor model */
    Robot r_j;
    double closest_dist = 10000;
    int S = 0;
    for (int j = 0; j < states_t.size(); j++){
        if (r_i.id == states_t[j].id){
            continue;
        }
        double dist = this->euclidean(r_i.position, states_t[j].position);
        // if (dist > this->sensing){ /* Take infinity time to converge */
        //     continue;
        // }
        if (this->sense2(r_i.position, states_t[j].position, this->deg2rad(15)) && dist < closest_dist){
        // if (this->sense(r_i.position, states_t[j].position, this->r) && dist < closest_dist){
            closest_dist = dist;
            r_j = states_t[j];
            if (r_j.type == r_i.type){
                S = 1;
            } else {
                S = 2;
            }
        }
    }

    Vector2 p;
    double V = this->vmax;
    double l =  0.1;//0.014;
    double dt = this->dt;
    switch(S){
        case 0: p = this->fwd_0(r_i.position, V, l, dt); break; 
        case 1: p = this->fwd_1(r_i.position, V, l, dt); break; 
        case 2: p = this->fwd_2(r_i.position, V, l, dt); break; 
    }
    /* Do not cross the world limits */
    p.x = std::max(std::min(this->worldsize-this->r * 2.0, p.x), -this->worldsize+this->r * 2.0);
    p.y = std::max(std::min(this->worldsize-this->r * 2.0, p.y), -this->worldsize+this->r * 2.0);

    /* Do not hit other robots */
    closest_dist = 10000;
    for (int j = 0; j < this->states.size(); j++){
        if (r_i.id == this->states[j].id){
            continue;
        }
        double dist = this->euclidean(p, this->states[j].position);
        if (dist < closest_dist){
            closest_dist = dist;
            r_j = this->states[j];
        }
    }
    double dx = r_j.position.x - p.x;
    double dy = r_j.position.y - p.y;
    double phi = atan2(dy, dx);
    double min_x = r_j.position.x + 2.0 * this->r * cos(phi+M_PI);
    double min_y = r_j.position.y + 2.0 * this->r * sin(phi+M_PI);
    double dist = this->euclidean(p, r_j.position);
    this->states[(int)r_i.id].position.theta = p.theta;
    if (dist <= (this->r * 2.0)){
        this->states[(int)r_i.id].position.x = min_x;
        this->states[(int)r_i.id].position.y = min_y;
    } else {
        this->states[(int)r_i.id].position.x = p.x;
        this->states[(int)r_i.id].position.y = p.y;
    }
    this->states[(int)r_i.id].velocity.x = cos(p.theta);
    this->states[(int)r_i.id].velocity.y = sin(p.theta);
}

void Controller::update(){
    std::vector<Robot> states_t;
    states_t = this->states;

    // #pragma omp parallel for ordered schedule(dynamic)
    #pragma omp parallel for
    for (int i = 0; i < this->robots; ++i){
        this->control(states_t[i], states_t);
    }

    ClusterMetric metric(this->robots, this->groups, 0.3);
    this->metric_v = metric.compute(this->states);

    if(this->logging){
        this->logfile << this->metric_v << "\n" ;
    }
}

int main(int argc, char **argv)
{
    srand(time(0));
    // ROS setups:
    ros::init(argc, argv, "minimalist_controller", ros::init_options::AnonymousName); // node name

    ros::NodeHandle nh("~"); // create a node handle; need to pass this to the class constructor

    ROS_INFO("[Main] Instantiating an object of type Controller");
    Controller control(&nh);

    long iterations = 0;
    bool cvok = true;
    int porcente = 0;
    long max_it = control.max_iteration;
    do{
        // auto start = std::chrono::high_resolution_clock::now(); 
        control.update();
        if (control.gui){
            cvok = control.draw(iterations);
        }
        if (!(iterations % (int)(max_it * 0.1))){
            // auto stop = std::chrono::high_resolution_clock::now(); 
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 
            // ROS_INFO("Update %f ms", (double)duration.count()/1000.0);
            ROS_INFO("Processing: %d", porcente*10);
            porcente += 1;
        }
        iterations += 1;
    }while(ros::ok() && (iterations < max_it) && cvok);
    if (control.gui){
        cv::waitKey(0);
    }

    return 0;
}