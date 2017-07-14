//
//  EwayIktest.cpp
//  
//
//  Created by Xuan Liu on 17/6/23.
//
//

#include "NumericalIkSolver.h"

#include "Eigen/Dense"
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <assert.h>


double testconstraint(std::vector<CNumericalIkSolver::Joint> robot, Eigen::MatrixXd m_MatTargetEndEffector)
{
    
//    std::cout<<robot[0].dAngle<<robot[1].dAngle<<robot[2].dAngle<<robot[3].dAngle<<robot[4].dAngle<<robot[5].dAngle<<robot[6].dAngle<<std::endl;
    
    bool bFlagEndEffectorPosition = false;
    bool bFlagEndEffectorOrientation = false;
    
    
    // Calculate vector distance between the result and the target.
    double dDistance = 0.0;
    
    
    // absolute endeff constraint
    for (int i = 0; i < 3; i++)
    {
        dDistance += std::abs(m_MatTargetEndEffector(i) - robot[6].MatEndEffector(i));
    }
    
    return dDistance;
}

int main(int argc, char** argv)
{
    //test
    google::InitGoogleLogging(argv[0]);
    
    Eigen::Vector3d Teff;
    Eigen::Vector3d Telbow;
    
    Teff << 0.45, 0.05, 0.55; // end_effector, target
    Telbow << 0, 1.0, -1.8; //elbow, temporarily doesn't work when causing the local minima
    
    CNumericalIkSolver::MyConstraintPtr p = testconstraint;
    
    
    CNumericalIkSolver example;
    example.InitRobot();
    example.RunSolver(Teff, Telbow, 1.0, 0.0, p);
    
    
    
    example.PrintResult();
    

    
    std::cout<<"Task finished!!!!!!!!"<<std::endl;
    
    return EXIT_SUCCESS;

}

