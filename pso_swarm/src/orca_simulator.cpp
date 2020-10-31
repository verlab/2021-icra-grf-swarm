#include "orca_simulator.h"
 
#ifdef _OPENMP
#include <omp.h>
#endif
 
ORCA_Simulator::ORCA_Simulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f)
{
    kdTree_ = new KdTree(this);
    //timeStep_ = 0.1;
}
 
ORCA_Simulator::ORCA_Simulator(float timeStep, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
{
    kdTree_ = new KdTree(this);
    defaultAgent_ = new Agent(this);
 
    defaultAgent_->maxNeighbors_ = maxNeighbors;
    defaultAgent_->maxSpeed_ = maxSpeed;
    defaultAgent_->neighborDist_ = neighborDist;
    defaultAgent_->radius_ = radius;
    defaultAgent_->timeHorizon_ = timeHorizon;
    defaultAgent_->timeHorizonObst_ = timeHorizonObst;
    defaultAgent_->velocity_ = velocity;
}
 
ORCA_Simulator::~ORCA_Simulator()
{
    if (defaultAgent_ != NULL) {
        delete defaultAgent_;
        defaultAgent_ = NULL;
    }
 
    for (int i = 0; i < agents_.size(); ++i) {
        delete agents_[i];
        agents_[i] = NULL;
    }
 
    for (int i = 0; i < obstacles_.size(); ++i) {
        delete obstacles_[i];
        obstacles_[i] = NULL;
    }
 
    delete kdTree_;
    kdTree_ = NULL;
}
 
int ORCA_Simulator::addAgent(float x, float y, int numFlocks)
{
    Agent *agent = new Agent(this);
 
    agent->id_ = agents_.size();
     
    agent->position_= Vector2(x, y);
     
 
    agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
    agent->maxSpeed_ = defaultAgent_->maxSpeed_;
    agent->neighborDist_ = defaultAgent_->neighborDist_;
    agent->radius_ = defaultAgent_->radius_;
    agent->timeHorizon_ = defaultAgent_->timeHorizon_;
    agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
    agent->velocity_ = defaultAgent_->velocity_;
    agent->memory_.resize(numFlocks);
    // for (int i = 0; i < (numFlocks); ++i){
    //     agent->memory_[i].first = 0;
    //     agent->memory_[i].second.set_x((rand() % 30/3.8) - 15/3.8);
    //     agent->memory_[i].second.set_y((rand() % 30/3.8) - 15/3.8);
    // }  
    agent->otherMemory_.first = 0;  
    agent->otherMemory_.second.set_x((rand() % 30/3.8) - 15/3.8);
    agent->otherMemory_.second.set_y((rand() % 30/3.8) - 15/3.8);
 
    agents_.push_back(agent);
 
    return agents_.size() - 1;
}
 
void ORCA_Simulator::setAgentPosition(int i){
    agents_[i]->position_= Vector2(agents_[i]->pose_.x(), agents_[i]->pose_.y());
    ros::spinOnce();
}
 
int ORCA_Simulator::addObstacle(const std::vector<Vector2> &vertices)
{
    const int obstacleNo = obstacles_.size();
 
    for (int i = 0; i < vertices.size(); ++i) {
        Obstacle *obstacle = new Obstacle();
        obstacle->point_ = vertices[i];
 
        if (i != 0) {
            obstacle->prevObstacle_ = obstacles_.back();
            obstacle->prevObstacle_->nextObstacle_ = obstacle;
        }
 
        if (i == vertices.size() - 1) {
            obstacle->nextObstacle_ = obstacles_[obstacleNo];
            obstacle->nextObstacle_->prevObstacle_ = obstacle;
        }
 
        obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);
 
        if (vertices.size() == 2) {
            obstacle->isConvex_ = true;
        }
        else {
            obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
        }
 
        obstacle->id_ = obstacles_.size();
 
        obstacles_.push_back(obstacle);
    }
 
    return obstacleNo;
}


void ORCA_Simulator::boidDoStep(int * vec)
{
    kdTree_->buildAgentTree();
    kdTree_->buildObstacleTree();

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        agents_[i]->computeNeighbors();
        agents_[i]->computeNewBoidVelocity(vec);
    }
    globalTime_ += timeStep_;
}


void ORCA_Simulator::PSODoStep(int numAgents, int flockSize)
{
    kdTree_->buildAgentTree();
    kdTree_->buildObstacleTree();

    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        agents_[i]->computeNeighbors();
        agents_[i]->computeNewPSOVelocity(numAgents, flockSize);
    }
    globalTime_ += timeStep_;
}


void ORCA_Simulator::orcaDoStep()
{
    #ifdef _OPENMP
        #pragma omp parallel for
    #endif
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        agents_[i]->computeNewVelocity();
    }

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        agents_[i]->update();
    }

    globalTime_ += timeStep_;
}
 
int ORCA_Simulator::mst(){
    int v = agents_.size();
    int visited[v], p[v];
    int current, totalvisited;
    float mincost, d[v];
 
    for(int i = 0; i< agents_.size(); i++){
        visited[i] = 0;
        d[i] = 1000.0f;
        p[i] = -1;
    }
 
    current = 0;
    d[current] = 0;
    totalvisited = 1;
    visited[current] = 1;
    while(totalvisited < v){
        for(int i = 0; i < v; i++){
            
            if(abs(agents_[current]->position_-agents_[i]->position_) != 0.0f && agents_[current]->flock_id_ == agents_[i]->flock_id_){
                if(visited[i] == 0){
                    if(d[i] > abs(agents_[current]->position_-agents_[i]->position_)){
                        d[i] = abs(agents_[current]->position_-agents_[i]->position_);
                        p[i] = current;
                    }
                }
            }
        }
        mincost = 32767.0f;
        for(int i = 0; i < v; i++){
            if(visited[i] == 0){
                if(d[i] < mincost){
                    mincost = d[i];
                    current = i;
                }
            }
        }
        visited[current] = 1;
        totalvisited++;
    }
    mincost = 0.0f;
    for(int i = 0; i < v; i++){
        mincost += d[i];
    }
     
    for(int i = 0; i < v; i++){
        if(p[i] != -1 && abs(agents_[i]->position_-agents_[p[i]]->position_) > 1.0f ){
            return 0;
        }
    }
    return 1;
}
 
 
int ORCA_Simulator::getAgentAgentNeighbor(int agentNo, int neighborNo) const
{
    return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
}
 
int ORCA_Simulator::getAgentMaxNeighbors(int agentNo) const
{
    return agents_[agentNo]->maxNeighbors_;
}
 
float ORCA_Simulator::getAgentMaxSpeed(int agentNo) const
{
    return agents_[agentNo]->maxSpeed_;
}
 
float ORCA_Simulator::getAgentNeighborDist(int agentNo) const
{
    return agents_[agentNo]->neighborDist_;
}
 
int ORCA_Simulator::getAgentNumAgentNeighbors(int agentNo) const
{
    return agents_[agentNo]->agentNeighbors_.size();
}
 
int ORCA_Simulator::getAgentNumObstacleNeighbors(int agentNo) const
{
    return agents_[agentNo]->obstacleNeighbors_.size();
}
 
int ORCA_Simulator::getAgentNumORCALines(int agentNo) const
{
    return agents_[agentNo]->orcaLines_.size();
}
 
int ORCA_Simulator::getAgentObstacleNeighbor(int agentNo, int neighborNo) const
{
    return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
}
 
const Line &ORCA_Simulator::getAgentORCALine(int agentNo, int lineNo) const
{
    return agents_[agentNo]->orcaLines_[lineNo];
}
 
const Vector2 &ORCA_Simulator::getAgentPosition(int agentNo) const
{
    return agents_[agentNo]->position_;
}
 
const Vector2 &ORCA_Simulator::getAgentGoal(int agentNo) const
{
    return agents_[agentNo]->goal_;
}
 
const Pose &ORCA_Simulator::getAgentPose(int agentNo) const
{
    return agents_[agentNo]->pose_;
}
 
const Vector2 &ORCA_Simulator::getAgentPrefVelocity(int agentNo) const
{
    return agents_[agentNo]->prefVelocity_;
}
 
float ORCA_Simulator::getAgentRadius(int agentNo) const
{
    return agents_[agentNo]->radius_;
}
 
float ORCA_Simulator::getAgentTimeHorizon(int agentNo) const
{
    return agents_[agentNo]->timeHorizon_;
}
 
float ORCA_Simulator::getAgentTimeHorizonObst(int agentNo) const
{
    return agents_[agentNo]->timeHorizonObst_;
}
 
const Vector2 &ORCA_Simulator::getAgentVelocity(int agentNo) const
{
    return agents_[agentNo]->velocity_;
}
 
float ORCA_Simulator::getGlobalTime() const
{
    return globalTime_;
}
 
int ORCA_Simulator::getNumAgents() const
{
    return agents_.size();
}
 
int ORCA_Simulator::getNumObstacleVertices() const
{
    return obstacles_.size();
}
 
const Vector2 &ORCA_Simulator::getObstacleVertex(int vertexNo) const
{
    return obstacles_[vertexNo]->point_;
}
 
int ORCA_Simulator::getNextObstacleVertexNo(int vertexNo) const
{
    return obstacles_[vertexNo]->nextObstacle_->id_;
}
 
int ORCA_Simulator::getPrevObstacleVertexNo(int vertexNo) const
{
    return obstacles_[vertexNo]->prevObstacle_->id_;
}
 
float ORCA_Simulator::getTimeStep() const
{
    return timeStep_;
}
 
void ORCA_Simulator::processObstacles()
{
    kdTree_->buildObstacleTree();
}
 
bool ORCA_Simulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
{
    return kdTree_->queryVisibility(point1, point2, radius);
}
 
void ORCA_Simulator::setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
{
    if (defaultAgent_ == NULL) {
        defaultAgent_ = new Agent(this);
    }
 
    defaultAgent_->maxNeighbors_ = maxNeighbors;
    defaultAgent_->maxSpeed_ = maxSpeed;
    defaultAgent_->neighborDist_ = neighborDist;
    defaultAgent_->radius_ = radius;
    defaultAgent_->timeHorizon_ = timeHorizon;
    defaultAgent_->timeHorizonObst_ = timeHorizonObst;
    defaultAgent_->velocity_ = velocity;
}
 
void ORCA_Simulator::setAgentMaxNeighbors(int agentNo, int maxNeighbors)
{
    agents_[agentNo]->maxNeighbors_ = maxNeighbors;
}
 
void ORCA_Simulator::setAgentMaxSpeed(int agentNo, float maxSpeed)
{
    agents_[agentNo]->maxSpeed_ = maxSpeed;
}
 
void ORCA_Simulator::setAgentNeighborDist(int agentNo, float neighborDist)
{
    agents_[agentNo]->neighborDist_ = neighborDist;
}
 
void ORCA_Simulator::setAgentPosition(int agentNo, const Vector2 &position)
{
    agents_[agentNo]->position_ = position;
}
 
void ORCA_Simulator::setAgentPrefVelocity(int agentNo, const Vector2 &prefVelocity)
{
    agents_[agentNo]->prefVelocity_ = prefVelocity;
}
 
void ORCA_Simulator::setAgentRadius(int agentNo, float radius)
{
    agents_[agentNo]->radius_ = radius;
}
 
void ORCA_Simulator::setAgentTimeHorizon(int agentNo, float timeHorizon)
{
    agents_[agentNo]->timeHorizon_ = timeHorizon;
}
 
void ORCA_Simulator::setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
{
    agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
}
 
void ORCA_Simulator::setAgentVelocity(int agentNo, const Vector2 &velocity)
{
    agents_[agentNo]->velocity_ = velocity;
}
 
void ORCA_Simulator::setTimeStep(float timeStep)
{
    timeStep_ = timeStep;
}