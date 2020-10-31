#include "grad_swarm.h"

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

    this->mass = 100.00;
    this->vmax = 1.0;
    this->dt = 0.02;

    ROS_INFO("%d %d %f", this->robots, this->groups, this->sensing);
    
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
        cv::namedWindow("Gradient-type Segregation", cv::WINDOW_AUTOSIZE);
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

    cv::imshow("Gradient-Segregation", board);
    #ifdef SAVE_FIGURES
        char filenanme[17];
        std::sprintf (filenanme, "image_%06d.png", step);
        cv::imwrite(filenanme, board);
    #endif
    return (cv::waitKey(1) != 27);
}


bool Controller::getIntersection(double r, Vector2 circle, Vector2 p1, Vector2 p2, Vector2& o1, Vector2& o2){
    // Convert p1 and p2 to be relative to circle; circle -> (0,0)
    p1.x -= circle.x;
    p1.y -= circle.y;

    p2.x -= circle.x;
    p2.y -= circle.y;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dr = sqrt(dx * dx + dy * dy);
    double D = p1.x * p2.y - p2.x * p1.y;
    double disc = (r * r) * (dr * dr) - (D * D);
    double sgn_dy = (dy >= 0) - (dy < 0);

    if (disc > 0.0){
        o1.x = (D*dy + sgn_dy * dx * sqrt(disc)) / (dr*dr);
        o2.x = (D*dy - sgn_dy * dx * sqrt(disc)) / (dr*dr);
        o1.y = (-D*dx + fabs(dy) * sqrt(disc)) / (dr*dr);
        o2.y = (-D*dx - fabs(dy) * sqrt(disc)) / (dr*dr);

        o1.x += circle.x;
        o2.x += circle.x;
        o1.y += circle.y;
        o2.y += circle.y;
        return true;
    }
    return false;

}

std::vector<std::vector<Robot>> Controller::getAllRobotsNeighborns(std::vector<Robot> agents){
    std::vector<std::vector<Robot>> neighbors;
    // #pragma omp parallel for
    for (int i = 0; i < this->robots; ++i){
        std::vector<Robot> ri;
        neighbors.push_back(ri);
    }
    

    // #pragma omp parallel for
    for (int i = 0; i < (this->robots - 1); ++i){
        for (int j = i+1; j < this->robots; ++j){
            double dist = this->euclidean(agents[i].position, agents[j].position);
            if (dist <= this->sensing){
                neighbors[j].push_back(agents[i]);
                neighbors[i].push_back(agents[j]);
            }
        }
    }
    return neighbors;
    
}

std::vector<Vector2> Controller::getObstaclesPoints(double sensing, Vector2 p){
    /* World
             p3 o------o p4
                |      |                            
                |      |
             p1 o------o p2
    */
    Vector2 p1;
    p1.x = -this->worldsize; p1.y = -this->worldsize;
    Vector2 p2;
    p2.x = this->worldsize; p2.y = -this->worldsize;
    Vector2 p3;
    p3.x = -this->worldsize; p3.y = this->worldsize;
    Vector2 p4;
    p4.x = this->worldsize; p4.y = this->worldsize;

    std::vector<Vector2> obstacles;

    Vector2 out1, out2;
    double res = 0.1;
    if (this->getIntersection(sensing, p, p1, p2, out1, out2)){
            obstacles.push_back(out1);
            obstacles.push_back(out2);
            // printf("12: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.x, out2.x)+res; i < std::max(out1.x, out2.x); i += res){
            Vector2 p;
            p.x = i;
            p.y = out1.y;
            obstacles.push_back(p);
        }
    }
    if (this->getIntersection(sensing, p, p1, p3, out1, out2)){
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("13: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.y, out2.y)+res; i < std::max(out1.y, out2.y); i += res){
            Vector2 p;
            p.x = out1.x;
            p.y = i;
            obstacles.push_back(p);
        }
    }
    if (this->getIntersection(sensing, p, p2, p4, out1, out2)){
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("24: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.y, out2.y)+res; i < std::max(out1.y, out2.y); i += res){
            Vector2 p;
            p.x = out1.x;
            p.y = i;
            obstacles.push_back(p);
        }
    }
    if (this->getIntersection(sensing, p, p3, p4, out1, out2)){
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("34: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = (std::min(out1.x, out2.x)+res); i < std::max(out1.x, out2.x); i += res){
            Vector2 p;
            p.x = i;
            p.y = out1.y;
            obstacles.push_back(p);
        }
    }
    return obstacles;
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
    Vector2 a;
    a.x = 0.0;
    a.y = 0.0;

    // Obstacle repulsion gradient Sum{dUs}
    std::vector<Vector2> obstacles = this->getObstaclesPoints(this->safezone, r_i.position);
    for (int i = 0; i < obstacles.size(); ++i){
        double dx = (r_i.position.x + this->dt * r_i.velocity.x) - obstacles[i].x;
        double dy = (r_i.position.y + this->dt * r_i.velocity.y) - obstacles[i].y;
        double r = sqrt(dx * dx + dy * dy);
        double alpha = 1.0;
        double epslon = 0.04;
        double epslon0 = 0.04;
        double r0 = 0.8;
        double q1 = 1.0;
        double q2 = 1.0;
        double dUs = 6.0 * epslon/(alpha - 6.0) * (alpha * std::pow(r0, 6)/std::pow(r, 7) - exp(alpha)/r0) - (q1 * q2)/(4 * M_PI * epslon0 * r);
        a.x += -(dUs * dx)/r;
        a.y += -(dUs * dy)/r;
    }


    Vector2 group_vrel;
    double group_mass = this->mass;
    /* Inter-agent potential */
    for (int i = 0; i < states_t.size(); i++){
        double dx = (r_i.position.x + this->dt * r_i.velocity.x) - states_t[i].position.x;
        double dy = (r_i.position.y + this->dt * r_i.velocity.y) - states_t[i].position.y;
        double r = sqrt(dx * dx + dy * dy);

        double I = 2.0 * (double)(r_i.type == states_t[i].type) - 1.0;
        // Don`t be reative to robot from different type if there is no collision risk
        if ((I < 0) && (r > this->safezone)){
            continue;
        }
        // Ust += this->coulombBuckinghamPotential(dist*0.8, 0.04, 0.04, 0.8, 1.0, 16.0*I, -1.0);
        double alpha = 1.0;
        double epslon = 0.04;
        double epslon0 = 0.04;
        double r0 = 0.8;
        double q1 = 16.0;
        double q2 = -1.0;
        double dUst = 6.0 * epslon/(alpha - 6.0) * (alpha * std::pow(r0, 6)/std::pow(r * 1.2, 7) - exp(alpha)/r0) - (q1 * I * (q2))/(4 * M_PI * epslon0 * r * 1.2);
        a.x += -(dUst * dx)/r;
        a.y += -(dUst * dy)/r;
        // Get the sum of the relative velocity of all my neighbor and they mass
        if (I > 0 && (r < this->sensing) && (r > this->safezone)){
            group_vrel.x += (states_t[i].acceleration.x - r_i.acceleration.x); 
            group_vrel.y += (states_t[i].acceleration.y - r_i.acceleration.y); 
            group_mass += this->mass;
        }
    }
    // Now compute the kinetic Energy using relative velocity
    group_vrel = this->saturation(group_vrel, 1.0);
    double group_speed = (group_vrel.x * group_vrel.x + group_vrel.y * group_vrel.y) + 1.0e-9;
    double my_speed = (r_i.acceleration.x * r_i.acceleration.x + r_i.acceleration.y * r_i.acceleration.y) + 1.0e-9;

    double dUk = group_mass * group_speed;
    a.x += -(dUk * group_vrel.x)/group_speed;
    a.y += -(dUk * group_vrel.y)/group_speed;  
    double dUkm = group_mass * (this->vmax - my_speed);
    a.x += -(dUkm * r_i.acceleration.x)/my_speed;
    a.y += -(dUkm * r_i.acceleration.y)/my_speed; 
    
    // Update Acceleration
    // a = this->saturation(a, 1.0);
    a.x = a.x * 0.1 + r_i.acceleration.x * 0.9;
    a.y = a.y * 0.1 + r_i.acceleration.y * 0.9;
    this->states[(int)r_i.id].acceleration = this->saturation(a, 1.0);

    // Update Velocity
    this->states[(int)r_i.id].velocity.x += a.x * this->dt;
    this->states[(int)r_i.id].velocity.y += a.y * this->dt;
    this->states[(int)r_i.id].velocity = this->saturation(this->states[(int)r_i.id].velocity, this->vmax);

    // Update Position
    this->states[(int)r_i.id].position.x += this->dt * this->states[(int)r_i.id].velocity.x;
    this->states[(int)r_i.id].position.y += this->dt * this->states[(int)r_i.id].velocity.y;
}

void Controller::update(long iterations){
    std::vector<Robot> states_t;
    states_t = this->states;
    std::vector<std::vector<Robot>> neighbors = this->getAllRobotsNeighborns(this->states);

    // #pragma omp parallel for ordered schedule(dynamic)
    #pragma omp parallel for
    for (int i = 0; i < this->robots; ++i){
        this->control(states_t[i], neighbors[i]);
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
    ros::init(argc, argv, "grad_controller", ros::init_options::AnonymousName); // node name

    ros::NodeHandle nh("~"); // create a node handle; need to pass this to the class constructor

    ROS_INFO("[Main] Instantiating an object of type Controller");
    Controller control(&nh);
    long iterations = 0;
    bool cvok = true;
    int porcente = 0;
    long max_it = control.max_iteration;
    do{
        // auto start = std::chrono::high_resolution_clock::now(); 
        control.update(iterations);
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