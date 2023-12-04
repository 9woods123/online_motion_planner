#include "hybrid_astar/smoother.h"

using namespace HybridAStar;


void Smoother::debugPath() {

    for(auto point:fourDpath)
    {
        std::cout<<"point== ("<<point.getX()<<","<<point.getY()<<","
        <<point.getZ()<<")  "<<std::endl;
    }

}

Eigen::Vector3d
Smoother::targetobstacleTerm(Eigen::Vector3d xi,
                             const HybridAStar::CollisionDetection *configurationSpace) {

    Eigen::Vector3d targetobstacleGradient;
    if(configurationSpace->getTargetObstacleGradient(xi,&targetobstacleGradient))
    {
        return -wtargetObstacle*targetobstacleGradient;
    }
    return Eigen::Vector3d(0,0,0);
}


Eigen::Vector3d Smoother::obstacleTerm(Eigen::Vector3d xi,
                                       const HybridAStar::CollisionDetection *configurationSpace){

    // TODO add a gradient calculated by considering the targets.
    Eigen::Vector3d obstacleGradient;
    double d_threshold=2.5;
    double distance;
    if(configurationSpace->
            getDistanceAndGradientAtPosition(xi, &distance,&obstacleGradient))
    {
        if(distance<d_threshold)
        {
            return -wObstacle*obstacleGradient;
        }

    }

    return Eigen::Vector3d(0,0,0);
}

float Smoother::FeasibleTerm(Eigen::Vector3d xim2, Eigen::Vector3d xim1,
                             Eigen::Vector3d xi,
                             Eigen::Vector3d xip1,Eigen::Vector3d xip2,
                             float t_im2,float t_im1,
                             float t_i,
                             float t_ip1, float t_ip2) {



}

Eigen::Vector3d Smoother::smoothnessTerm(Eigen::Vector3d xim3, Eigen::Vector3d xim2, Eigen::Vector3d xim1,
                                         Eigen::Vector3d xi, Eigen::Vector3d xip1, Eigen::Vector3d xip2,
                                         Eigen::Vector3d xip3) {

    Eigen::Vector3d gradient= wAccSmooth * (-xim3+ 6*xim2 - 15*xim1 + 20*xi - 15*xip1 + 6*xip2 -xip3)+
                              wVelSmooth * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
                              
    return gradient;

}



void  Smoother::tracePath(const Node4D* node, int i, std::vector<Node4D> path )
{
    if (node == nullptr) {
        std::reverse(path.begin(),path.end());
        this->fourDpath =path;
        return;
    }

    i++;
    path.push_back(*node);
    tracePath(node->getPred(), i, path);
}


bool Smoother::getPolynomialTraj(const mav_msgs::EigenTrajectoryPoint::Vector& waypoints, 
    mav_trajectory_generation::Trajectory* trajectory,std::vector<double> segment_times) 
 {

 if (waypoints.size() < 2) {
    return false;
  }

  constexpr int N = 10;
  constexpr int D = 3;
  mav_trajectory_generation::PolynomialOptimization<N> poly_opt(D);

  int num_vertices = waypoints.size();

  int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::ACCELERATION;

  mav_trajectory_generation::Vertex::Vector vertices(
      num_vertices, mav_trajectory_generation::Vertex(D));


  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.front().position_W);
  vertices.front().addConstraint(
            mav_trajectory_generation::derivative_order::VELOCITY,
            waypoints.front().velocity_W);
 //   TODO the  VELOCITY should be  

  vertices.back().makeStartOrEnd(0, derivative_to_optimize);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.back().position_W);

  // Now do the middle bits.
  size_t j = 1;
  for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
    vertices[j].addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        waypoints[i].position_W);
    j++;
  }


  poly_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  if (poly_opt.solveLinear()) {
    poly_opt.getTrajectory(trajectory);
  } else {
    return false;
  }
  return true;

 }

void Smoother::printPathInfo(const std::vector<Node4D> &path,
                                            const std::vector<double> &new_times) {


        for (int i = 0; i < path.size() ; ++i) {
        std::cout <<new_times[i]<<" ";
        }
    std::cout<<"==========print path info======"<<path.size()<<std::endl;
    for (int i = 1; i < path.size() - 1; ++i) {
        // x i minus 2 x_(i+2)
        Eigen::Vector3d xim1(path[i - 1].getX(), path[i - 1].getY(), path[i - 1].getZ());
        Eigen::Vector3d xi(path[i].getX(), path[i].getY(), path[i].getZ());
        Eigen::Vector3d xip1(path[i + 1].getX(), path[i + 1].getY(), path[i + 1].getZ());


        double deltaX_i = getDistance(xip1, xi);
        double deltaX_im1 = getDistance(xi, xim1);
        double delta_v_i,delta_v_im1;

        double t_i = new_times[i];
        double t_ip1 = new_times[i + 1];
        double t_im1 = new_times[i - 1];

        //===================continous velocity penalty===================
        if (i == path.size() - 2) {

            double v_i_keep_v_im1 = deltaX_im1 / (t_i - t_im1);
            delta_v_i=2*(deltaX_i-v_i_keep_v_im1*(t_ip1-t_i))/(t_ip1-t_i);

            Eigen::Vector3d xim2(path[i - 2].getX(), path[i - 2].getY(),path[i - 2].getZ());
            double deltaX_im2=getDistance(xim1,xim2);
            double t_im2 = new_times[i - 2];
            double v_im1_keep_v_im2 = deltaX_im2/(t_im1-t_im2);
            delta_v_im1=2*(deltaX_im1-v_im1_keep_v_im2*(t_i-t_im1))/(t_i-t_im1);


        } else if (i == 1) {

            double v_i_keep_v_im1 = deltaX_im1 / (t_i - t_im1);
            delta_v_i=2*(deltaX_i-v_i_keep_v_im1*(t_ip1-t_i))/(t_ip1-t_i);

            double v_im1_keep_v_im2 = 0;
            delta_v_im1=2*(deltaX_im1-v_im1_keep_v_im2*(t_i-t_im1))/(t_i-t_im1);

//                std::cout <<"delta_v_0:"<<delta_v_im1<<" delta_v_1:"<<delta_v_i<<std::endl;


        } else {


            double v_i_keep_v_im1 = deltaX_im1 / (t_i - t_im1);
            delta_v_i=2*(deltaX_i-v_i_keep_v_im1*(t_ip1-t_i))/(t_ip1-t_i);

            Eigen::Vector3d xim2(path[i - 2].getX(), path[i - 2].getY(),path[i - 2].getZ());
            double deltaX_im2=getDistance(xim1,xim2);
            double t_im2 = new_times[i - 2];
            double v_im1_keep_v_im2 = deltaX_im2/(t_im1-t_im2);
            delta_v_im1=2*(deltaX_im1-v_im1_keep_v_im2*(t_i-t_im1))/(t_i-t_im1);


//                std::cout <<"delta_v_im1:"<<delta_v_im1<<" delta_v_i:"<<delta_v_i<<std::endl;

        }

        std::cout<<"v_"<<i-1<<": "<<deltaX_im1/(t_i-t_im1)<<
        "  a_"<<i-1<<": "<< delta_v_im1/(t_i-t_im1)<<std::endl;
    }
    std::cout<<"==========print over=========="<<std::endl;
}

std::vector<double> Smoother::adjustPathTime(std::vector<Node4D> &path,
                                             double v_init, double v_max, double a_max) {

    double v_end=0;

    std::cout <<"v_init:"<<v_init<<" v_max:"<<v_max<<" a_max:"<<a_max<<std::endl;

    v_init=v_init>v_max ? v_max:v_init;

    std::vector<double> init_segemntTimes=getSegmentTimes(&path,v_init,v_max,a_max);

    std::vector<double> times;
    times.push_back(0);      //front    0s

    for (int i = 1; i < path.size() ; ++i) {
        double t_i=times[i-1]+init_segemntTimes[i];
        times.push_back(t_i);
    }
//     times.back()+=8;
     //debug



    std::vector<double> new_times=times;
    std::vector<double> temp_times=times;

    int iterations=0;
    int maxIterations=100;


    while (iterations < maxIterations) {
//        std::cout<<"============a new inter for time adjustment============"<<std::endl;
        // choose the first three nodes of the path
        for (int i = 1; i < path.size() - 1; ++i) {
            // x i minus 2 x_(i+2)
            Eigen::Vector3d xim1(path[i - 1].getX(), path[i - 1].getY(),path[i - 1].getZ());
            Eigen::Vector3d xi(path[i].getX(), path[i].getY(),path[i].getZ());
            Eigen::Vector3d xip1(path[i + 1].getX(), path[i + 1].getY(),path[i + 1].getZ());

            double gradient = 0;

            double deltaX_i =getDistance(xip1,xi);
            double deltaX_im1 =getDistance(xi,xim1);

            double t_i = new_times[i];
            double t_ip1 = new_times[i + 1];
            double t_im1 = new_times[i - 1];

            gradient += getVelPenaltyGradient(deltaX_i, deltaX_im1, t_i, t_ip1, t_im1, v_max);
            //===================continous velocity penalty===================

            double delta_v_i,delta_v_im1;

//            std::cout <<"deltaX_i:"<<deltaX_i<<" deltaX_im1:"<<deltaX_im1<<std::endl;

            if (i == path.size() - 2) {

                double v_i_keep_v_im1 = deltaX_im1 / (t_i - t_im1);
                delta_v_i=2*(deltaX_i-v_i_keep_v_im1*(t_ip1-t_i))/(t_ip1-t_i);
                delta_v_i=v_end-v_i_keep_v_im1;

                Eigen::Vector3d xim2(path[i - 2].getX(), path[i - 2].getY(),path[i - 2].getZ());
                double deltaX_im2=getDistance(xim1,xim2);
                double t_im2 = new_times[i - 2];
                double v_im1_keep_v_im2 = deltaX_im2/(t_im1-t_im2);
                delta_v_im1=2*(deltaX_im1-v_im1_keep_v_im2*(t_i-t_im1))/(t_i-t_im1);


            } else if (i == 1) {

                double v_i_keep_v_im1 = deltaX_im1 / (t_i - t_im1);
                delta_v_i=2*(deltaX_i-v_i_keep_v_im1*(t_ip1-t_i))/(t_ip1-t_i);

                double v_im1_keep_v_im2 = v_init;
                delta_v_im1=2*(deltaX_im1-v_im1_keep_v_im2*(t_i-t_im1))/(t_i-t_im1);

//                std::cout <<"delta_v_0:"<<delta_v_im1<<" delta_v_1:"<<delta_v_i<<std::endl;


            } else {


                double v_i_keep_v_im1 = deltaX_im1 / (t_i - t_im1);
                delta_v_i=2*(deltaX_i-v_i_keep_v_im1*(t_ip1-t_i))/(t_ip1-t_i);

                Eigen::Vector3d xim2(path[i - 2].getX(), path[i - 2].getY(),path[i - 2].getZ());
                double deltaX_im2=getDistance(xim1,xim2);
                double t_im2 = new_times[i - 2];
                double v_im1_keep_v_im2 = deltaX_im2/(t_im1-t_im2);
                delta_v_im1=2*(deltaX_im1-v_im1_keep_v_im2*(t_i-t_im1))/(t_i-t_im1);


//                std::cout <<"delta_v_im1:"<<delta_v_im1<<" delta_v_i:"<<delta_v_i<<std::endl;

            }

            gradient += 1*getAccPenaltyGradient(delta_v_i, delta_v_im1, t_i,t_ip1, t_im1,a_max);
            gradient+= 4*getAccSmoothGradient(delta_v_i, delta_v_im1, t_i,t_ip1, t_im1,a_max);
            //===================continous Acc penalty===================

            gradient += getTimeBarrierGradient(t_i, t_ip1, t_im1);

            gradient = std::max(-0.1, std::min(gradient, 0.1));

            double newTi = t_i- gradient;

            temp_times[i] = newTi;

        }

        new_times=temp_times;
        iterations++;
    }
     std::cout <<"================== before opti=============="<<std::endl;
     printPathInfo(path,times);

    std::vector<double> befo_segmentTimes;
    for (size_t i = 1; i < times.size(); ++i) {
        float diff = times[i] - times[i - 1];
        befo_segmentTimes.push_back(diff);
    }

    std::cout <<"==================segmentTimes=============="<<std::endl;
    for (int i = 0; i < befo_segmentTimes.size() ; ++i) {
        std::cout <<befo_segmentTimes[i]<<" ";
    }


     std::cout <<"================== after opti=============="<<std::endl;
     printPathInfo(path,new_times);

    std::vector<double> segmentTimes;
    for (size_t i = 1; i < new_times.size(); ++i) {
        float diff = new_times[i] - new_times[i - 1];
        segmentTimes.push_back(diff);
    }

    std::cout <<"==================segmentTimes=============="<<std::endl;
    for (int i = 0; i < segmentTimes.size() ; ++i) {
        std::cout <<segmentTimes[i]<<" ";
    }
    std::cout <<"================================================="<<std::endl;

return segmentTimes;

}


float Smoother::getVelPenaltyGradient(double deltaXK, double deltaXKim1, double tK, double tKp1, double tKim1, float v_max)
{

    double gradient = 0;

    double penaltyConstraint = deltaXK / (tKp1 - tK) - v_max;
    if (penaltyConstraint > 0) {
        gradient += penaltyConstraint / std::pow((tKp1 - tK), 2);
    }

    double penaltyConstraint2 = deltaXKim1 / (tK - tKim1) - v_max;
    if (penaltyConstraint2 > 0) {
        gradient -= penaltyConstraint2 / std::pow((tK - tKim1), 2);
    }

//        std::cout<<"vel penalty_constraint="<<penaltyConstraint<<std::endl;
//        std::cout<<"vel penalty_constraint2="<<penaltyConstraint2<<std::endl;
//        std::cout<<"vel gradient="<<gradient<<std::endl;


    return gradient;
}

float Smoother::getAccSmoothGradient(double delta_v_k, double delta_v_kim1,
                                     double t_k, double t_kp1, double t_kim1,
                                     double a_max) {
    double gradient = 0;

    double smooGrad=2*(
            std::pow(delta_v_k,2)/std::pow((t_kp1-t_k),3)
            - std::pow(delta_v_kim1,2)/std::pow((t_k-t_kim1),3)
    );

    gradient+=1*smooGrad;

    return gradient;
}

float Smoother::getAccPenaltyGradient(double delta_v_k, double delta_v_kim1,
                                      double t_k, double t_kp1, double t_kim1,double a_max) {

//    gradient += 3*getAccPenaltyGradient(delta_v_i, delta_v_im1, t_i,t_ip1, t_im1,a_max);

        double gradient = 0;

        double penaltyConstraint = std::abs(delta_v_k / (t_kp1 - t_k)) - a_max;
        if (penaltyConstraint > 0) {
            gradient += penaltyConstraint / std::pow((t_kp1 - t_k), 2);
        }

        double penaltyConstraint2 = std::abs(delta_v_kim1 / (t_k - t_kim1)) - a_max;
        if (penaltyConstraint2 > 0) {
            gradient -= penaltyConstraint2 / std::pow((t_k - t_kim1), 2);
        }


//
//        std::cout << "Acc penalty_constraint=" << penaltyConstraint << std::endl;
//        std::cout << "Acc penalty_constraint2=" << penaltyConstraint2 << std::endl;
//        std::cout << "Acc gradient=" << gradient << std::endl;

        return gradient;
    }

float Smoother::getTimeBarrierGradient(double tK, double tKp1, double tKim1) {
    double gradient = 0;

    double barrier1 = tKp1 - tK;
    gradient += 1 / std::pow((tKp1 - tK), 2);

    double barrier2 = tK - tKim1;
    gradient -= 1 / std::pow((tK - tKim1), 2);

    return 1e-5 * gradient;
}

std::vector<double> Smoother:: getSegmentTimes(std::vector<Node4D>* path, double v_init,double v_max, double a_max)
{
        // Calculate the segments times by assuming that  
        // accelerate at maximum acceleration, and  maintain a constant speed 
        // at v_max
//        std::cout<<"v_init="<<v_init<<" v_max="<<v_max<<" a_max="<<a_max<<std::endl;
//        std::cout<<"delta_times== ";

        std::vector<double> times;
        double v_last=v_init;
        for (int i=0; i<path->size()-1;i++)
        {
            double delta_time=0;
            double min_acc_time( (v_max-v_last)/a_max  );
            double distance_acc= 
            v_last*min_acc_time+0.5* a_max*min_acc_time*min_acc_time;
            double step_distance=std::sqrt(
                                  SQ(path->at(i+1).getX()-path->at(i).getX())
                                 +SQ(path->at(i+1).getY()-path->at(i).getY())
                                 +SQ(path->at(i+1).getZ()-path->at(i).getZ()));


            if (distance_acc<step_distance)
            {
                delta_time=min_acc_time+(step_distance-distance_acc)/v_max;
                v_last=v_max;
                //index= (index==0) ? i : index;
            }
            else
            {
                double a=a_max/2; double b=v_last; double c=-step_distance;
                delta_time= (-b+std::sqrt(b*b-4*a*c) )/ (2 * a);
                v_last=v_last+delta_time*a_max;
                if(v_last>v_max){v_last=v_max;}
            }
//            std::cout<<" d_time: "<<delta_time<<std::endl;
            times.push_back(delta_time);
        }

    return times;
}













 bool Smoother::smooth(const HybridAStar::CollisionDetection *configurationSpace,double d_t,
                       double v_init ,double v_max, double a_max) {

     auto start = std::chrono::high_resolution_clock::now();

     int iterations = 0;
     // the maximum iterations for the gd smoother
     int maxIterations = 100;
     // the lenght of the path in number of nodes
     int pathLength = fourDpath.size();
     if(pathLength<5){ return false;}
     std::vector<Node4D> newPath = fourDpath;

     // ================== an  optimization calculating by adjust the wapypoint===================
     float totalWeight = wAccSmooth + wVelSmooth +  wObstacle + wtargetObstacle;
     std::vector<Node4D> temp_path=newPath;
     while (iterations < maxIterations) {
         // choose the first three nodes of the path
         for (int i = 3; i < pathLength - 3; ++i) {
             // x i minus 2 x_(i+2)

             Eigen::Vector3d xim3(newPath[i - 3].getX(), newPath[i - 3].getY(), newPath[i - 3].getZ());
             Eigen::Vector3d xim2(newPath[i - 2].getX(), newPath[i - 2].getY(),newPath[i - 2].getZ());
             Eigen::Vector3d xim1(newPath[i - 1].getX(), newPath[i - 1].getY(),newPath[i - 1].getZ());
             Eigen::Vector3d xi(newPath[i].getX(), newPath[i].getY(),newPath[i].getZ());
             Eigen::Vector3d xip1(newPath[i + 1].getX(), newPath[i + 1].getY(),newPath[i + 1].getZ());
             Eigen::Vector3d xip2(newPath[i + 2].getX(), newPath[i + 2].getY(),newPath[i + 2].getZ());
             Eigen::Vector3d xip3(newPath[i + 3].getX(), newPath[i + 3].getY(), newPath[i + 3].getZ());
             Eigen::Vector3d correction;

             correction = correction + smoothnessTerm(xim3, xim2, xim1, xi, xip1, xip2, xip3);
             correction = correction + obstacleTerm(xi,configurationSpace);
             correction = correction + targetobstacleTerm(xi,configurationSpace);

             //  gradient descent
             xi = xi - alpha * correction/totalWeight;

             temp_path[i].setX(xi[0]);
             temp_path[i].setY(xi[1]);
             temp_path[i].setZ(xi[2]);
         }
         newPath=temp_path;
         iterations++;
     }

     //================== an  optimization calculating by adjust the wapypoint===================


     //====================== using  minimum snap to get a Polynomial Traj .======================
     std::vector<double> segment_times= Smoother::getSegmentTimes(&newPath,  v_init, v_max,  a_max);
//     std::vector<double> segment_times= Smoother::adjustPathTime(newPath,  v_init, v_max,  a_max);


     mav_msgs::EigenTrajectoryPoint::Vector waypoints;
     for(auto point:newPath)
     {
         mav_msgs::EigenTrajectoryPoint wp;
         wp.position_W<<point.getX(),point.getY(),point.getZ();
         waypoints.push_back(wp);
     }

     mav_trajectory_generation::Trajectory  trajectory;
     bool success=getPolynomialTraj(waypoints,&trajectory,segment_times);
     mav_msgs::EigenTrajectoryPointVector path;
     if (success) {
         mav_trajectory_generation::sampleWholeTrajectory(trajectory, d_t, &path);
     }
    //====================== using  minimum snap to get a Polynomial Traj .======================

    //=======================  adjust trajectory yaw ==============================
    std::vector<Node4D> finalPath;
    for(auto wp:path)
    {
        Node4D newnode;
        newnode.setX(wp.position_W[0]);
        newnode.setY(wp.position_W[1]);
        newnode.setZ(wp.position_W[2]);
        finalPath.push_back(newnode);
    }

    finalPath.back().setT(fourDpath.back().getT());

    for (int i = 0; i < finalPath.size()-1 ; ++i)
    {
        Eigen::Vector3d xi(finalPath[i].getX(), finalPath[i].getY(), finalPath[i].getZ());
        Eigen::Vector3d xip1(finalPath[i+1].getX(), finalPath[i+1].getY(), finalPath[i+1].getZ());
        Eigen::Vector3d Dxi = xip1 - xi;
        finalPath[i].setT(std::atan2(Dxi.y(), Dxi.x()));
    }
     fourDpath = finalPath;
    //=======================  adjust trajectory yaw ==============================

     auto end = std::chrono::high_resolution_clock::now();
     std::chrono::duration<double, std::milli> elapsed = end - start;
     std::cout << "trajectory optimer cost==" << elapsed.count() << " ms" << std::endl;
     return true;

 }

inline float  Smoother::getDistance(Eigen::Vector3d from, Eigen::Vector3d to) {
    return (from-to).norm();
}
