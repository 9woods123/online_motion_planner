#include "hybrid_astar/node4d.h"


//const float Node4D::dy[] = { 0,         -0.0415893, 0.0415893, 0 ,        0                     };
//const float Node4D::dx[] = { 0.7068582, 0.705224,   0.705224,  0.705224 , 0.705224             };
//const float Node4D::dz[] = { 0,         0,          0,         0.155893,  -0.155893            };
//const float Node4D::dt[] = { 0,         0.1178097,  -0.1178097,0,         0                     };
//const float Node4D::dy[] = { 0,              -delta_y,   delta_y,   0 ,         0                     };
//const float Node4D::dx[] = { delta_distance, delta_x,    delta_x,   delta_x ,   delta_x            };
//const float Node4D::dz[] = { 0,              0,          0,         0.10,       -0.10             };
//const float Node4D::dt[] = { 0,              delta_t,    -delta_t,  0,          0                     };
using namespace HybridAStar;
const int Node4D::dir = 5;
float Node4D::radius_min=0;
float Node4D::step_min=1;
float Node4D::dx[dir]={};
float Node4D::dy[dir]={};
float Node4D::dz[dir]={};
float Node4D::dt[dir]={};

void copy_array( float a[],float b[] ,int size)
{
    for (int i = 0; i < size; ++i) {
        a[i] = b[i];
    }
}

void Node4D::setMotionPrims() {

    float delta_x=Node4D::step_min;
    float delta_t= asin(delta_x/Node4D::radius_min);
    float delta_y= (1- cos(delta_t))*Node4D::radius_min;
    float delta_distance=delta_t*Node4D::radius_min;
    float dy_array[]=   { 0,              -delta_y,   delta_y,  0 ,         0               };
    float dx_array[]  = { delta_distance, delta_x,    delta_x,  delta_x ,   delta_x         };
    float dz_array[]  = { 0,              0,          0,        delta_x/5,  -delta_x/5      };
    float dt_array[]  = { 0,              -delta_t,   delta_t,  0,          0               };



    copy_array(dx,dx_array,dir);

    copy_array(dy,dy_array,dir);

    copy_array(dz,dz_array,dir);

    copy_array(dt,dt_array,dir);



}





//Node4D* Node4D::createSuccessor(const int i) {
//
//    ///   -2 ，2 respresents the movement along z-axis., -1 respresents 2k+1 -1 .
//
//  int n=  (Node4D::dir-2-1)/2;            // when dir =5, n=2;
//  float u_r,u_t,u_x,u_y,u_z,movement;
//
//  u_r= -radius_min+float(i/n)*radius_min;
//  movement=1;
//  u_t= movement/u_r;                     // dt*r= arc length （movement）
//  u_x=u_r * sin(u_t);
//  u_y=u_r-u_r*cos(u_t);
//  u_z=0;
//
//  if(i==Node4D::dir-2){
//      u_x=movement;
//      u_t=0;
//      u_y=0;
//      u_z=0.2;
//  }
//  if(i==Node4D::dir-1){
//        u_x=movement;
//        u_t=0;
//        u_y=0;
//        u_z=-0.2;
//    }
//
//  float xSucc;
//  float ySucc;
//  float zSucc;
//  float tSucc;
//
//  // calculate successor positions forward
//  if (i < Node4D::dir) {                           // =====woods=====这里是5，因为考虑了z轴运动
//    xSucc = x + u_x * cos(t) - u_y * sin(t);
//    ySucc = y + u_x * sin(t) + u_y * cos(t);
//    zSucc = z + u_z;                    //=========woods======= 这里没有考虑伏仰角度
//    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
//  }
//  // backwards
//  else {
//    xSucc = x - dx[i - Node4D::dir] * cos(t) - dy[i - Node4D::dir] * sin(t);
//    ySucc = y - dx[i - Node4D::dir] * sin(t) + dy[i - Node4D::dir] * cos(t);
//    zSucc = z - dz[i - Node4D::dir];                 //=========woods======= 这里没有考虑伏仰角度
//    tSucc = Helper::normalizeHeadingRad(t - dt[i - Node4D::dir]);
//  }
//  return new Node4D(xSucc, ySucc, zSucc,tSucc, g, 0, this, i);
//}


Node4D* Node4D::createSuccessor(const int i) {


  float xSucc;
  float ySucc;
  float zSucc;
  float tSucc;

  // calculate successor positions forward
  if (i < Node4D::dir) {                           // =====woods=====这里是5，因为考虑了z轴运动
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    zSucc = z + dz[i];                    //=========woods======= 这里没有考虑伏仰角度
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else {
    xSucc = x - dx[i - Node4D::dir] * cos(t) - dy[i - Node4D::dir] * sin(t);
    ySucc = y - dx[i - Node4D::dir] * sin(t) + dy[i - Node4D::dir] * cos(t);
    zSucc = z - dz[i - Node4D::dir];                 //=========woods======= 这里没有考虑伏仰角度
    tSucc = Helper::normalizeHeadingRad(t - dt[i - Node4D::dir]);
  }
  return new Node4D(xSucc, ySucc, zSucc,tSucc, g, 0, this, i);
}

//                                      MOVEMENT COST
void Node4D::updateG() {
  // forward driving
  if (prim < Node4D::dir) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > Node4D::dir-1) {              //=======woods======前一个节点是倒车的化
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning;
      }
    } else {

      g += dx[0];
      g +=std::abs(dz[prim]);

    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < Node4D::dir) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } else {

      g += dx[0] * Constants::penaltyReversing;
      g +=std::abs(dz[prim]);
    }
  }
}

//                                 4D NODE COMPARISON
bool Node4D::operator == (const Node4D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (int)z == (int)rhs.z&&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
