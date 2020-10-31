#include "agent.h"
 
 
Agent::Agent(ORCA_Simulator *sim) : maxNeighbors_(0), maxSpeed_(0.0f), neighborDist_(0.0f), radius_(0.0f), sim_(sim), timeHorizon_(0.0f), timeHorizonObst_(0.0f) {
}
 
void Agent::setFlockID(int f){
    flock_id_ = f;
}
 
 
void Agent::computeNeighbors()
{
    obstacleNeighbors_.clear();
    float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
    sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);
 
    agentNeighbors_.clear();
 
    if (maxNeighbors_ > 0) {
        rangeSq = sqr(neighborDist_);
        sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
    }
}
 
//Calcular velocidade preferencial para o FL-ORCA
void Agent::computeNewBoidVelocity(int *vec)
{
    v1_ = Vector2(0.0f, 0.0f);
    v2_ = Vector2(0.0f, 0.0f);
    v3_ = Vector2(0.0f, 0.0f);
    leader_position_ = position_;
    neighbor_ = Vector2(0.0f, 0.0f);
    other_flock_ = 0;
    free_vision_ = 1; follower_ = 0; turn_right_ = 0;
    int nv1_ = 0, nv2_ = 0, nv3_ = 0;
    int leader = 0;
    int neighbor_other_group = 0;

    Vector2 v = Vector2(0.0f, 0.0f);
    Vector2 u = Vector2(0.0f, 0.0f);

    
    for (int i = 0; i < agentNeighbors_.size(); ++i) {
        const Agent *const other = agentNeighbors_[i].second;
        if(other->other_flock_ == 1 and other->flock_id_ == flock_id_){
            neighbor_other_group = 1;
        }
    }

    //Manter-se proximo aos vizinhos
    for (int i = 0; i < agentNeighbors_.size(); ++i) {

        const Agent *const other = agentNeighbors_[i].second;
        float view = atan2((other->position_.y() - position_.y()) , (other->position_.x() - position_.x()))*(180/M_PI);

        v = goal_ - position_;
        u = other->position_ - position_;

        float ang = (v.x()*u.x() + v.y()*u.y())/(abs(v)*abs(u));

        if(ang > 1.0){
            ang = 1.0;
        }
        if(ang < -1.0){
            ang = -1.0;
        }

        ang = acos(ang)*(180/M_PI);

        //Coesao
        if(other->flock_id_ == flock_id_){
            v1_ = v1_ + (other->position_);
            nv1_++;
        }
        else if(other->flock_id_ != flock_id_ && ang < 90 && abs(other->position_ - position_) < 3.0f/3.8){
            other_flock_ = 1;
        }

        //Afastamento
        if(other->flock_id_ != flock_id_ && ang < 45 && abs(other->position_ - position_) < 1.5f/3.8){
            v2_ = v2_ - (other->position_ - position_);
            nv2_++;
        }

        //Alinhamento
        if(other->flock_id_ == flock_id_){
            v3_ = v3_ + (other->velocity_);
            nv3_++;
            vec[id_]++;
        }


        //Teste de visão livre
        if(ang < 20 && abs(other->position_ - position_) < 10.0f/3.8 ){
            free_vision_ = 0;
        }

        //Teste de seguidor
        if(other->flock_id_ == flock_id_ && free_vision_ == 0 && (other->free_vision_ == 1 || other->follower_ == 1)){

            if(other->free_vision_ == 1){
                follower_ = 1;
            }
            else if(other->follower_ >= 1){
                follower_ = 2;
            }
            neighbor_ += other->position_;
            leader++;

            if(other->free_vision_ == 1 || (abs(leader_position_-goal_) > abs(other->position_-goal_))){
                leader_position_ = other->position_;
            }

        }

        //Vire a direita
        if((other->other_flock_ == 1 || other_flock_== 1) && other->flock_id_ == flock_id_ &&
            free_vision_ == 0 && follower_ == 0 && other->follower_ == 0){
            turn_right_ = 1;
        }
    }

    if(nv1_ > 0){
        v1_ = v1_/nv1_;
        v1_ = v1_ - position_;
    }


    if(nv2_++ > 0){
        v2_ = v2_/nv2_;
    }
    //Evitar o espelhamento de movimentos
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    v2_ = (((0.2*r)+1)*v2_);
    v2_ = v2_;


    if(nv3_ > 0){
        v3_ = v3_/nv3_;
    }

    pVel_ = prefVelocity_;


    if (absSq(v1_) > 1.0f) {
        v1_ = normalize(v1_);
    }
    if (absSq(v2_) > 1.0f) {
        v2_ = normalize(v2_);
    }
    if (absSq(v3_) > 1.0f) {
        v3_ = normalize(v3_);
    }
    if (absSq(prefVelocity_) > 1.0f) {
        prefVelocity_ = normalize(prefVelocity_);
    }

    if(other_flock_ == 1){
        //Visao Livre
        if(free_vision_==1){
            flockVel_ = 15*v1_ + 0.5*v2_ + 5*v3_ + 0.35f*prefVelocity_;
        }
        //Seguidor
        else if(follower_> 0){
            flockVel_ = 15*v1_ + 0.5*v2_ + 50*(leader_position_ - position_);
        }
        //Outro grupo
        else if(turn_right_ == 1){
            Vector2 z = Vector2((position_.x()-(position_.y()-goal_.y())), (position_.y()+(position_.x()-goal_.x())));
            flockVel_ = 50*v1_ + 0.5*v2_ + 1*v3_ + 15*(z-position_);
        }
        else{
            flockVel_ = 50*v1_ + 0.5*v2_ + 5*v3_;
        }

        prefVelocity_ = flockVel_;

        if(absSq(prefVelocity_) > 1.0f){
            prefVelocity_ = normalize(prefVelocity_);
        }
    }
    //Agente nao enxerga outro grupo
    else{
        //Vire a direita
        if(turn_right_== 1){
            printf("Vire a direita\n");
            Vector2 z = Vector2((position_.x()-(position_.y()-goal_.y())), (position_.y()+(position_.x()-goal_.x())));
            flockVel_ = 15*v1_ + 2*v3_ + 50*(z-position_);
            prefVelocity_ = (prefVelocity_ + flockVel_);
        }
        //Seguidor
        else if(follower_ > 0){
            printf("Seguidor\n");
            flockVel_ = 10*v1_ + 50*(leader_position_ - position_);
            prefVelocity_ = (prefVelocity_ + flockVel_);
        }
        //Visao Livre
        else{
            printf("Else\n");
            flockVel_ = (v1_ + v3_);
            prefVelocity_ = (prefVelocity_ + flockVel_);
        }
        if (absSq(prefVelocity_) > 1.0f) {
            prefVelocity_ = normalize(prefVelocity_);
        }
    }

    if (absSq(prefVelocity_) > 1.0f) {
        prefVelocity_ = normalize(prefVelocity_);
    }
}


void Agent::computeNewPSOVelocity(int numAgents, int numFlocks){
    numNeighbors_ = 0;
    
    Vector2 prevVel = velocity_;
    Vector2 rep;
    int nrep = 0;
    int gBestNeighbors = 0;
    
    std::pair<int, Vector2> neighborhood_memory[numFlocks];
    for (int i = 0; i < (numFlocks); ++i){
        neighborhood_memory[i].first = 0;
    }    

    for (int i = 0; i < agentNeighbors_.size(); ++i){
 
        const Agent *const other = agentNeighbors_[i].second;     
        // test neighbornhood
        if ((abs(other->position_ - position_) > neighborDist_)){
            printf("This shouldn't be right!\n");
            continue;
        }
        
        //agente do mesmo grupo
        // if(other->flock_id_ == flock_id_){
        if((other->flock_id_ == flock_id_) && (abs(other->position_ - position_) < neighborDist_)){
            numNeighbors_++;
            
            if(other->numNeighbors_ >= gBestNeighbors){
                gBestNeighbors = other->numNeighbors_;
                gBestPosition_ = other->position_;                
            }

            //atualizacao da memoria sobre outros grupos a partir de mensagens
            //for(int j = 0; j< (numFlocks); j++){
            //    if(other->memory_[j].first > memory_[j].first){
            //        memory_[j].first = other->memory_[j].first;
            //        memory_[j].second = other->memory_[j].second;
            //    }
            //}  
        }

        //agente de grupo diferente
        else{

            neighborhood_memory[other->flock_id_].first++;
            neighborhood_memory[other->flock_id_].second = other->position_;

            if(abs(other->position_ - position_) < neighborDist_){
                rep = rep - (other->position_ - position_);
                nrep++;
            }
            
            if(other->memory_[flock_id_].first > otherMemory_.first && abs(other->memory_[flock_id_].second - position_) > neighborDist_){
                emptyMemory_ = 0;
                otherMemory_.first = other->memory_[flock_id_].first;
                otherMemory_.second = other->memory_[flock_id_].second;
            }          
        }

        //atualizacao da informacao de memoria do agente (o que ELE viu sobre os outros grupos)
        if(neighborhood_memory[other->flock_id_].first > memory_[other->flock_id_].first){
            memory_[other->flock_id_].first = neighborhood_memory[other->flock_id_].first;
            memory_[other->flock_id_].second = neighborhood_memory[other->flock_id_].second;
        }


        //atualizacao da memoria sobre outros grupos a partir de mensagens
        for(int j = 0; j< (numFlocks); j++){
            if(other->memory_[j].first > memory_[j].first){
                memory_[j].first = other->memory_[j].first;
                memory_[j].second = other->memory_[j].second;
            }
        }    
    }   





    //limpar memoria
    //if(static_cast<int>(sim_->globalTime_)%50 == 0){
    if(abs(otherMemory_.second- position_) <= 0.5/3.8){
        emptyMemory_ = 1;
        otherMemory_.first = 0;
        //otherMemory_.second.set_x((rand() % 30/3) - 15/3.8);
        //otherMemory_.second.set_y((rand() % 30/3) - 15/3.8);

        otherMemory_.second.set_x(position_.x());
        otherMemory_.second.set_y(position_.y());
    }


    //atualizar gBest
    if(static_cast<int>(sim_->globalTime_)%50 == 0 || abs(gBestPosition_- position_) <= 0.5/3.8){
        if(numNeighbors_ == 0){
            gBestPosition_.set_x((rand() % 30/3.8) - 15/3.8);
            gBestPosition_.set_y((rand() % 30/3.8) - 15/3.8);
        }

        //printf("Alterei a memoria!\n");
    }

    goal_ = gBestPosition_;

    if(goal_.x() < -16/3.8){
        goal_.set_x((rand() % 30/3.8) - 15/3.8);
    }
    if(goal_.x() > 16/3.8){
        goal_.set_x((rand() % 30/3.8) - 15/3.8);
    }
    if(goal_.y() < -16/3.8){
        goal_.set_y((rand() % 30/3.8) - 15/3.8);        
    }
    if(goal_.y() > 16/3.8){
        goal_.set_y((rand() % 30/3.8) - 15/3.8);
    }


    goalVector_ = goal_ - position_;
    
    if(nrep > 0){
        rep = rep/nrep;
    }

    Vector2 other_memory = otherMemory_.second - position_;

    
    //=============================================================
    //=============================================================
    //=============================================================
    //=============================================================


    //CALCULO DA VELOCIDADE PREFERENCIAL

    //agente nao enxerga nenhum outro agente
    if(agentNeighbors_.size() == 0){
        if(abs(goal_ - position_) < 0.5/3.8){
            goal_.set_x((rand() % 30/3.8) - 15/3.8);
            goal_.set_y((rand() % 30/3.8) - 15/3.8);
        }

        if(emptyMemory_== 1){
            prefVelocity_ = goalVector_ + prevVel;
        }
        else{
            goalVector_ = other_memory;
            prefVelocity_ = other_memory + prevVel;
        }



    }
    else{
        //agente so enxerga vizinhos de outros grupos
        if(numNeighbors_ == 0){

            if(abs(other_memory- position_) > neighborDist_ && other_memory.x() != 0.0 && other_memory.y() != 0.0){
                
                if(emptyMemory_== 1){
                    if(nrep < 5){
                        prefVelocity_ = goalVector_;
                    }else{
                        prefVelocity_ = goalVector_ + rep;
                    }
                }
                else{
                    //prefVelocity_ = other_memory + rep;
                    //testar essas linhas no lugar da de cima
                    if(nrep < 5){
                        prefVelocity_ = other_memory;
                    }else{
                        prefVelocity_ = other_memory + rep;
                    }
                }
            }
            
            else{
                if(static_cast<int>(sim_->globalTime_)%10 == 0){
                    goal_.set_x((rand() % 30/3.8) - 15/3.8);
                    goal_.set_y((rand() % 30/3.8) - 15/3.8);
                    Vector2 goalVector_ = goal_ - position_;
                }
                //prefVelocity_ = goalVector_ + other_memory+ 10*prevVel; 
                 
                if(emptyMemory_== 1){
                    
                    if(nrep < 5){
                        prefVelocity_ = goalVector_ + prevVel;
                    }else{
                        prefVelocity_ = goalVector_ + prevVel + rep;
                    }

                }
                else{
                    prefVelocity_ = goalVector_ + other_memory + prevVel + rep;
                }
            }
        }
        else{
            if(emptyMemory_== 1){
                prefVelocity_ = (goalVector_ + rep)/numNeighbors_ + (7/numNeighbors_)*prevVel;
            }
            else{            
                prefVelocity_ = (goalVector_ + other_memory + rep)/numNeighbors_ + (7/numNeighbors_)*prevVel;
            }

            if(numNeighbors_<=2){

                if(rand()%1 < 0.5){
                    if(rand()%1 < 0.5){
                        prefVelocity_ += Vector2(rand()%1, -rand()%1);
                    }                    
                    else{
                        prefVelocity_ += Vector2(rand()%2, rand()%1);
                    }
                }
                else{
                    if(rand()%1 < 0.5){
                        prefVelocity_ += (Vector2(-rand()%1, -rand()%1) - goalVector_);
                    }
                    else{
                        prefVelocity_ += (Vector2(-rand()%2, rand()%1)- goalVector_);
                    }
                }
            }       
        }
    }
        
    if (absSq(prefVelocity_) > 1.0f) {
        prefVelocity_ = normalize(prefVelocity_);
    }

}

 
void Agent::update()
{
    velocity_ = newVelocity_;
 
    //Comando para simulacao com OpenCV
    position_ += velocity_ * (sim_->timeStep_);
    
    heading_ = atan2(goal_.y()-position_.y(), goal_.x()-position_.x())*(180/M_PI);
}

  
/* Search for the best new velocity. */
void Agent::computeNewVelocity()
{
    orcaLines_.clear();
 
    const float invTimeHorizonObst = 1.0f / timeHorizonObst_;
 
    /* Create obstacle ORCA lines. */
    for (int i = 0; i < obstacleNeighbors_.size(); ++i) {
 
        const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
        const Obstacle *obstacle2 = obstacle1->nextObstacle_;
 
        const Vector2 relativePosition1 = obstacle1->point_ - position_;
        const Vector2 relativePosition2 = obstacle2->point_ - position_;
 
        /*
         * Check if velocity obstacle of obstacle is already taken care of by
         * previously constructed obstacle ORCA lines.
         */
        bool alreadyCovered = false;
 
        for (int j = 0; j < orcaLines_.size(); ++j) {
            if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -ORCA_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -ORCA_EPSILON) {
                alreadyCovered = true;
                break;
            }
        }
 
        if (alreadyCovered) {
            continue;
        }
 
        /* Not yet covered. Check for collisions. */
 
        const float distSq1 = absSq(relativePosition1);
        const float distSq2 = absSq(relativePosition2);
 
        const float radiusSq = sqr(radius_);
 
        const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
        const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
        const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
 
        Line line;
 
        if (s < 0.0f && distSq1 <= radiusSq) {
            /* Collision with left vertex. Ignore if non-convex. */
            if (obstacle1->isConvex_) {
                line.point = Vector2(0.0f, 0.0f);
                line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
                orcaLines_.push_back(line);
            }
 
            continue;
        }
        else if (s > 1.0f && distSq2 <= radiusSq) {
            /* Collision with right vertex. Ignore if non-convex
             * or if it will be taken care of by neighoring obstace */
            if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
                line.point = Vector2(0.0f, 0.0f);
                line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
                orcaLines_.push_back(line);
            }
 
            continue;
        }
        else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
            /* Collision with obstacle segment. */
            line.point = Vector2(0.0f, 0.0f);
            line.direction = -obstacle1->unitDir_;
            orcaLines_.push_back(line);
            continue;
        }
 
        /*
         * No collision.
         * Compute legs. When obliquely viewed, both legs can come from a single
         * vertex. Legs extend cut-off line when nonconvex vertex.
         */
 
        Vector2 leftLegDirection, rightLegDirection;
 
        if (s < 0.0f && distSqLine <= radiusSq) {
            /*
             * Obstacle viewed obliquely so that left vertex
             * defines velocity obstacle.
             */
            if (!obstacle1->isConvex_) {
                /* Ignore obstacle. */
                continue;
            }
 
            obstacle2 = obstacle1;
 
            const float leg1 = std::sqrt(distSq1 - radiusSq);
            leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
            rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
        }
        else if (s > 1.0f && distSqLine <= radiusSq) {
            /*
             * Obstacle viewed obliquely so that
             * right vertex defines velocity obstacle.
             */
            if (!obstacle2->isConvex_) {
                /* Ignore obstacle. */
                continue;
            }
 
            obstacle1 = obstacle2;
 
            const float leg2 = std::sqrt(distSq2 - radiusSq);
            leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
            rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
        }
        else {
            /* Usual situation. */
            if (obstacle1->isConvex_) {
                const float leg1 = std::sqrt(distSq1 - radiusSq);
                leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
            }
            else {
                /* Left vertex non-convex; left leg extends cut-off line. */
                leftLegDirection = -obstacle1->unitDir_;
            }
 
            if (obstacle2->isConvex_) {
                const float leg2 = std::sqrt(distSq2 - radiusSq);
                rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
            }
            else {
                /* Right vertex non-convex; right leg extends cut-off line. */
                rightLegDirection = obstacle1->unitDir_;
            }
        }
 
        /*
         * Legs can never point into neighboring edge when convex vertex,
         * take cutoff-line of neighboring edge instead. If velocity projected on
         * "foreign" leg, no constraint is added.
         */
 
        const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;
 
        bool isLeftLegForeign = false;
        bool isRightLegForeign = false;
 
        if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
            /* Left leg points into obstacle. */
            leftLegDirection = -leftNeighbor->unitDir_;
            isLeftLegForeign = true;
        }
 
        if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
            /* Right leg points into obstacle. */
            rightLegDirection = obstacle2->unitDir_;
            isRightLegForeign = true;
        }
 
        /* Compute cut-off centers. */
        const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position_);
        const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position_);
        const Vector2 cutoffVec = rightCutoff - leftCutoff;
 
        /* Project current velocity on velocity obstacle. */
 
        /* Check if current velocity is projected on cutoff circles. */
        const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
        const float tLeft = ((velocity_ - leftCutoff) * leftLegDirection);
        const float tRight = ((velocity_ - rightCutoff) * rightLegDirection);
 
        if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
            /* Project on left cut-off circle. */
            const Vector2 unitW = normalize(velocity_ - leftCutoff);
 
            line.direction = Vector2(unitW.y(), -unitW.x());
            line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
            orcaLines_.push_back(line);
            continue;
        }
        else if (t > 1.0f && tRight < 0.0f) {
            /* Project on right cut-off circle. */
            const Vector2 unitW = normalize(velocity_ - rightCutoff);
 
            line.direction = Vector2(unitW.y(), -unitW.x());
            line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
            orcaLines_.push_back(line);
            continue;
        }
 
        /*
         * Project on left leg, right leg, or cut-off line, whichever is closest
         * to velocity.
         */
        const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + t * cutoffVec)));
        const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
        const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (rightCutoff + tRight * rightLegDirection)));
 
        if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
            /* Project on cut-off line. */
            line.direction = -obstacle1->unitDir_;
            line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
            orcaLines_.push_back(line);
            continue;
        }
        else if (distSqLeft <= distSqRight) {
            /* Project on left leg. */
            if (isLeftLegForeign) {
                continue;
            }
 
            line.direction = leftLegDirection;
            line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
            orcaLines_.push_back(line);
            continue;
        }
        else {
            /* Project on right leg. */
            if (isRightLegForeign) {
                continue;
            }
 
            line.direction = -rightLegDirection;
            line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
            orcaLines_.push_back(line);
            continue;
        }
    }
 
    const int numObstLines = orcaLines_.size();
 
    const float invTimeHorizon = 1.0f / timeHorizon_;
 
    /* Create agent ORCA lines. */
    for (int i = 0; i < agentNeighbors_.size(); ++i) {
        const Agent *const other = agentNeighbors_[i].second;
 
        const Vector2 relativePosition = other->position_ - position_;
        const Vector2 relativeVelocity = velocity_ - other->velocity_;
        const float distSq = absSq(relativePosition);
        const float combinedRadius = radius_ + other->radius_;
        const float combinedRadiusSq = sqr(combinedRadius);
 
        Line line;
        Vector2 u;
 
        if (distSq > combinedRadiusSq) {
            /* No collision. */
            const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
            /* Vector from cutoff center to relative velocity. */
            const float wLengthSq = absSq(w);
 
            const float dotProduct1 = w * relativePosition;
 
            if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
                /* Project on cut-off circle. */
                const float wLength = std::sqrt(wLengthSq);
                const Vector2 unitW = w / wLength;
 
                line.direction = Vector2(unitW.y(), -unitW.x());
                u = (combinedRadius * invTimeHorizon - wLength) * unitW;
            }
            else {
                /* Project on legs. */
                const float leg = std::sqrt(distSq - combinedRadiusSq);
 
                if (det(relativePosition, w) > 0.0f) {
                    /* Project on left leg. */
                    line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                }
                else {
                    /* Project on right leg. */
                    line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                }
 
                const float dotProduct2 = relativeVelocity * line.direction;
 
                u = dotProduct2 * line.direction - relativeVelocity;
            }
        }
        else {
            /* Collision. Project on cut-off circle of time timeStep. */
            const float invTimeStep = 1.0f / sim_->timeStep_;
 
            /* Vector from cutoff center to relative velocity. */
            const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
 
            const float wLength = abs(w);
            const Vector2 unitW = w / wLength;
 
            line.direction = Vector2(unitW.y(), -unitW.x());
            u = (combinedRadius * invTimeStep - wLength) * unitW;
        }
 
        line.point = velocity_ + 0.5f * u;
        orcaLines_.push_back(line);
    }
 
    int lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);
 
    if (lineFail < orcaLines_.size()) {
        linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
    }
}
 
void Agent::insertAgentNeighbor(const Agent *agent, float &rangeSq)
{
    if (this != agent) {
        const float distSq = absSq(position_ - agent->position_);
 
        if (distSq < rangeSq) {
            if (agentNeighbors_.size() < maxNeighbors_) {
                agentNeighbors_.push_back(std::make_pair(distSq, agent));
            }
 
            int i = agentNeighbors_.size() - 1;
 
            while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
                agentNeighbors_[i] = agentNeighbors_[i - 1];
                --i;
            }
 
            agentNeighbors_[i] = std::make_pair(distSq, agent);
 
            if (agentNeighbors_.size() == maxNeighbors_) {
                rangeSq = agentNeighbors_.back().first;
            }
        }
    }
}
 
void Agent::insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq)
{
    const Obstacle *const nextObstacle = obstacle->nextObstacle_;
 
    const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position_);
 
    if (distSq < rangeSq) {
        obstacleNeighbors_.push_back(std::make_pair(distSq, obstacle));
 
        int i = obstacleNeighbors_.size() - 1;
 
        while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
            obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
            --i;
        }
 
        obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
    }
}
 
 
bool linearProgram1(const std::vector<Line> &lines, int lineNo, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
{
    const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
    const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);
 
    if (discriminant < 0.0f) {
        /* Max speed circle fully invalidates line lineNo. */
        return false;
    }
 
    const float sqrtDiscriminant = std::sqrt(discriminant);
    float tLeft = -dotProduct - sqrtDiscriminant;
    float tRight = -dotProduct + sqrtDiscriminant;
 
    for (int i = 0; i < lineNo; ++i) {
        const float denominator = det(lines[lineNo].direction, lines[i].direction);
        const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);
 
        if (std::fabs(denominator) <= ORCA_EPSILON) {
            /* Lines lineNo and i are (almost) parallel. */
            if (numerator < 0.0f) {
                return false;
            }
            else {
                continue;
            }
        }
 
        const float t = numerator / denominator;
 
        if (denominator >= 0.0f) {
            /* Line i bounds line lineNo on the right. */
            tRight = std::min(tRight, t);
        }
        else {
            /* Line i bounds line lineNo on the left. */
            tLeft = std::max(tLeft, t);
        }
 
        if (tLeft > tRight) {
            return false;
        }
    }
 
    if (directionOpt) {
        /* Optimize direction. */
        if (optVelocity * lines[lineNo].direction > 0.0f) {
            /* Take right extreme. */
            result = lines[lineNo].point + tRight * lines[lineNo].direction;
        }
        else {
            /* Take left extreme. */
            result = lines[lineNo].point + tLeft * lines[lineNo].direction;
        }
    }
    else {
        /* Optimize closest point. */
        const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);
 
        if (t < tLeft) {
            result = lines[lineNo].point + tLeft * lines[lineNo].direction;
        }
        else if (t > tRight) {
            result = lines[lineNo].point + tRight * lines[lineNo].direction;
        }
        else {
            result = lines[lineNo].point + t * lines[lineNo].direction;
        }
    }
 
    return true;
}
 
int linearProgram2(const std::vector<Line> &lines, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
{
    if (directionOpt) {
        /*
         * Optimize direction. Note that the optimization velocity is of unit
         * length in this case.
         */
        result = optVelocity * radius;
    }
    else if (absSq(optVelocity) > sqr(radius)) {
        /* Optimize closest point and outside circle. */
        result = normalize(optVelocity) * radius;
    }
    else {
        /* Optimize closest point and inside circle. */
        result = optVelocity;
    }
 
    for (int i = 0; i < lines.size(); ++i) {
        if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
            /* Result does not satisfy constraint i. Compute new optimal result. */
            const Vector2 tempResult = result;
 
            if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
                result = tempResult;
                return i;
            }
        }
    }
 
    return lines.size();
}
 
void linearProgram3(const std::vector<Line> &lines, int numObstLines, int beginLine, float radius, Vector2 &result)
{
    float distance = 0.0f;
 
    for (int i = beginLine; i < lines.size(); ++i) {
        if (det(lines[i].direction, lines[i].point - result) > distance) {
            /* Result does not satisfy constraint of line i. */
            std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));
 
            for (int j = numObstLines; j < i; ++j) {
                Line line;
 
                float determinant = det(lines[i].direction, lines[j].direction);
 
                if (std::fabs(determinant) <= ORCA_EPSILON) {
                    /* Line i and line j are parallel. */
                    if (lines[i].direction * lines[j].direction > 0.0f) {
                        /* Line i and line j point in the same direction. */
                        continue;
                    }
                    else {
                        /* Line i and line j point in opposite direction. */
                        line.point = 0.5f * (lines[i].point + lines[j].point);
                    }
                }
                else {
                    line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                }
 
                line.direction = normalize(lines[j].direction - lines[i].direction);
                projLines.push_back(line);
            }
 
            const Vector2 tempResult = result;
 
            if (linearProgram2(projLines, radius, Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, result) < projLines.size()) {
                /* This should in principle not happen.  The result is by definition
                 * already in the feasible region of this linear program. If it fails,
                 * it is due to small floating point error, and the current result is
                 * kept.
                 */
                result = tempResult;
            }
 
            distance = det(lines[i].direction, lines[i].point - result);
        }
    }
}