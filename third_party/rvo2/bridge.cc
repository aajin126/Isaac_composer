// Thin C ABI wrapper for official RVO2, shared by ROS Python and Isaac Python.
#include "RVO.h"
#include <cmath>
#include <exception>
#include <algorithm>

extern "C" {
void* crowd_rvo_create(float dt) {
  auto* sim = new RVO::RVOSimulator();
  sim->setTimeStep(dt);
  return sim;
}
void crowd_rvo_destroy(void* ptr) { delete static_cast<RVO::RVOSimulator*>(ptr); }
int crowd_rvo_agent(void* ptr, float horizon, float obstacleHorizon, float neighbors,
                    float radius, float obstacleRadius, int external) {
  auto* sim=static_cast<RVO::RVOSimulator*>(ptr);
  const auto id=sim->addAgent(RVO::Vector2(),neighbors,128,horizon,obstacleHorizon,radius,1.5F);
  sim->setAgentExternallyControlled(id,external!=0);
  sim->setAgentObstacleRadius(id,obstacleRadius); // Extra clearance for animation root-motion tracking.
  return static_cast<int>(id);
}
void crowd_rvo_segment(void* ptr, float ax, float ay, float bx, float by) {
  auto* sim=static_cast<RVO::RVOSimulator*>(ptr);
  sim->addObstacle({RVO::Vector2(ax,ay),RVO::Vector2(bx,by)});
}
void crowd_rvo_process(void* ptr) { static_cast<RVO::RVOSimulator*>(ptr)->processObstacles(); }
int crowd_rvo_query(void* ptr, int agent, const double* objective, int directionOpt, double* result) {
  auto* sim=static_cast<RVO::RVOSimulator*>(ptr);
  RVO::Vector2 velocity;
  const bool ok=sim->queryAgentVelocity(agent,RVO::Vector2(objective[0],objective[1]),directionOpt!=0,velocity);
  result[0]=velocity.x();result[1]=velocity.y();
  return ok ? 1 : 0;
}
void crowd_rvo_parameters(void* ptr, int agent, double* result) {
  auto* sim=static_cast<RVO::RVOSimulator*>(ptr);
  result[0]=sim->getAgentTimeHorizon(agent);
  result[1]=sim->getAgentRadius(agent);
  result[2]=sim->getAgentNeighborDist(agent);
}
void crowd_rvo_responsibilities(void* ptr, int count, const double* shares) {
  auto* sim=static_cast<RVO::RVOSimulator*>(ptr);
  for(int i=0;i<count;++i) {
    std::vector<float> row;
    if(shares) row.assign(shares+i*count,shares+(i+1)*count);
    sim->setAgentAvoidanceResponsibilities(i,row);
  }
}
float crowd_rvo_responsibility(void* ptr, int agent, int other) {
  return static_cast<RVO::RVOSimulator*>(ptr)->getAgentAvoidanceResponsibility(agent,other);
}
int crowd_rvo_neighbors(void* ptr, int agent, int* result, int capacity) {
  auto* sim=static_cast<RVO::RVOSimulator*>(ptr);
  const int count=std::min(static_cast<int>(sim->getAgentNumAgentNeighbors(agent)),capacity);
  for(int k=0;k<count;++k) result[k]=static_cast<int>(sim->getAgentAgentNeighbor(agent,k));
  return count;
}
void crowd_rvo_step(void* ptr, int count, float dt, const double* pos, const double* vel,
                    const double* pref, const double* radii, const double* speed,
                    double* result, int* feasible) {
  auto* sim=static_cast<RVO::RVOSimulator*>(ptr);
  sim->setTimeStep(dt);
  for(int i=0;i<count;++i) {
    sim->setAgentPosition(i,RVO::Vector2(pos[2*i],pos[2*i+1]));
    sim->setAgentVelocity(i,RVO::Vector2(vel[2*i],vel[2*i+1]));
    sim->setAgentPrefVelocity(i,RVO::Vector2(pref[2*i],pref[2*i+1]));
    sim->setAgentRadius(i,radii[i]);
    sim->setAgentMaxSpeed(i,speed[i]);
  }
  sim->doStep();
  for(int i=0;i<count;++i) {
    const auto v=sim->getAgentVelocity(i);
    result[2*i]=v.x();result[2*i+1]=v.y();feasible[i]=1;
    for(std::size_t k=0;k<sim->getAgentNumORCALines(i);++k) {
      const auto& line=sim->getAgentORCALine(i,k);
      if(RVO::det(line.direction,line.point-v)>1e-4F) feasible[i]=0;
    }
  }
}
}
