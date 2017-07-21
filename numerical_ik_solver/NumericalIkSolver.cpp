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

#include "ceres/dynamic_autodiff_cost_function.h"
#include "ceres/dynamic_numeric_diff_cost_function.h"

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <assert.h>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;

using std::vector;
using std::cout;
using std::endl;
using std::string;
using std::abs;

using ceres::NumericDiffCostFunction;
using ceres::DynamicNumericDiffCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CENTRAL;
using ceres::FORWARD;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;



Vector3d CNumericalIkSolver::m_MatEndEffector;
Vector3d CNumericalIkSolver::m_MatElbow;
Vector3d CNumericalIkSolver::m_MatEndEffOffset;

double CNumericalIkSolver::m_dCostTolerance;
double CNumericalIkSolver::m_dConstraintTolerance;
double CNumericalIkSolver::m_dCost;

const double CNumericalIkSolver::m_dPI = 3.141592653589793238;

int CNumericalIkSolver::m_iIterationTolerance;
int CNumericalIkSolver::m_iJointNum;
int CNumericalIkSolver::m_iInitFlag;
int CNumericalIkSolver::m_iArmFlag;

bool CNumericalIkSolver::m_bFlag;

double * CNumericalIkSolver::m_dAngles;


vector<CNumericalIkSolver::Joint> CNumericalIkSolver::m_vRobotArm;

CNumericalIkSolver::MyObjectivePtr CNumericalIkSolver::Obj;
CNumericalIkSolver::MyConstraintPtr CNumericalIkSolver::Constraint;


double CNumericalIkSolver::DefaultConstraint()
{
    
    // Calculate vector distance between the result and the target.
    double dDistance = 0.0;
    
    // absolute endeff constraint
    for (int i = 0; i < 3; i++)
    {
        dDistance += abs(m_MatTargetEndEffector(i) - m_MatEndEffector(i));
    }
    
    return dDistance;
}

int CNumericalIkSolver::CQuadCostFunctor::Satisfied(const double * const dAngles) const
{
    assert(m_iInitFlag == 0);
    
    double dTemp;
    for (int i = 0; i < m_iJointNum; i++){ // euint?
        dTemp = dAngles[i];
        if (dAngles[i] < -m_dPI || dAngles[i] > m_dPI) // normalization !! be careful here, change to [-pi,pi]
        {
            dTemp = dAngles[i] - (double)((int)(dAngles[i]/(2*m_dPI)))*2*m_dPI; // +1
            if (dTemp > m_dPI)
                dTemp -= 2*m_dPI;
            if (dTemp < -m_dPI)
                dTemp += 2*m_dPI;
        }
        
//        cout<<temp<<","<<robot[i+1].lb<<","<<robot[i+1].ub<<endl;
        if (dTemp < m_vRobotArm[i+1].dLowBound || dTemp > m_vRobotArm[i+1].dUpBound)
        {
//            cout<<temp<<","<<robot[i+1].lb<<","<<robot[i+1].ub<<endl;
            return FAIL;
        }
    }
    return SUCCESS;
}

int CNumericalIkSolver::InitRobot(int iArmFlag) //need OS params, change to loop add according to the size, need checking method to make sure the correctness of input data?
{
    
    //inner initialization e.g.
    Joint sJoint;
    
    sJoint.iIndex = 1;
    sJoint.strName = "1-base";
    sJoint.iParent = 0;
    sJoint.dAngle = 0.0;
    sJoint.MatAxis << 0.0,0.0,1.0;
    sJoint.MatOffset << 0.0,0.0,0.0;
    sJoint.MatPosition << 0.0,0.0,0.83;
    sJoint.dLowBound = 0.0;
    sJoint.dUpBound = 0.5;
    sJoint.MatOrientation.setIdentity();
    cout<<sJoint.dLowBound<<","<<sJoint.dUpBound<<endl;
    
    m_vRobotArm.push_back(sJoint);
    
    sJoint.iIndex = 2; // L1
    sJoint.strName = "2-joint";
    sJoint.iParent = 1;
    sJoint.dAngle = 0.0;
    sJoint.MatAxis << 0.0,1.0,0.0;
    sJoint.MatOffset << 0.0,0.0932,0.0;
    sJoint.dLowBound = -2.62;
    sJoint.dUpBound = 2.62;
    sJoint.MatOrientation.setIdentity();
    cout<<sJoint.dLowBound<<","<<sJoint.dUpBound<<endl;
    
    m_vRobotArm.push_back(sJoint);
    
    sJoint.iIndex = 3; // L2
    sJoint.strName = "3-joint";
    sJoint.iParent = 2;
    sJoint.dAngle = 0.0;
    sJoint.MatAxis << 0.0,0.0,1.0;
    sJoint.MatOffset << 0.0,0.0937,0.0;
    sJoint.dLowBound = -1.92;
    sJoint.dUpBound = m_dPI/2;
    sJoint.MatOrientation.setIdentity();
    cout<<sJoint.dLowBound<<","<<sJoint.dUpBound<<endl;
    
    m_vRobotArm.push_back(sJoint);
    
    sJoint.iIndex = 4; //L3
    sJoint.strName = "4-joint";
    sJoint.iParent = 3;
    sJoint.dAngle = 0.0;
    sJoint.MatAxis << 0.0,1.0,0.0;
    sJoint.MatOffset << 0.0,0.1685,0.0;
    sJoint.dLowBound = -2.62;
    sJoint.dUpBound = 2.62;
    sJoint.MatOrientation.setIdentity();
    cout<<sJoint.dLowBound<<","<<sJoint.dUpBound<<endl;
    
    m_vRobotArm.push_back(sJoint);
    
    sJoint.iIndex = 5; //L4
    sJoint.strName = "5-joint";
    sJoint.iParent = 4;
    sJoint.dAngle = 0.0;
    sJoint.MatAxis << 1.0,0.0,0.0;
    sJoint.MatOffset << 0.0,0.0608,0.0;
    sJoint.dLowBound = -0.96;
    sJoint.dUpBound = 2.12;
    sJoint.MatOrientation.setIdentity();
    cout<<sJoint.dLowBound<<","<<sJoint.dUpBound<<endl;
    
    m_vRobotArm.push_back(sJoint);
    
    sJoint.iIndex = 6; // L5
    sJoint.strName = "6-joint";
    sJoint.iParent = 5;
    sJoint.dAngle = 0.0;
    sJoint.MatAxis << 0.0,1.0,0.0;
    sJoint.MatOffset << 0.0,0.1722,-0.0108;
    sJoint.dLowBound = -2.62;
    sJoint.dUpBound = 2.62;
    sJoint.MatOrientation.setIdentity();
    cout<<sJoint.dLowBound<<","<<sJoint.dUpBound<<endl;
    
    m_vRobotArm.push_back(sJoint);
    
    sJoint.iIndex = 7; // L6
    sJoint.strName = "7-elbow";
    sJoint.iParent = 6;
    sJoint.dAngle = 0.0;
    sJoint.MatAxis << 0.0,0.0,1.0;
    sJoint.MatOffset << 0.0,0.083,0.0;
    sJoint.dLowBound = -1.70;
    sJoint.dUpBound = 1.70;
    sJoint.MatOrientation.setIdentity();
    cout<<sJoint.dLowBound<<","<<sJoint.dUpBound<<endl;
    
    m_vRobotArm.push_back(sJoint);
    
    //endeffoffset!!
    m_MatEndEffOffset << 0.0,0.131,0.0;
    
    //angles
    m_iJointNum = m_vRobotArm.size() - 1;
    m_dAngles = (double *)malloc(sizeof(double) * m_iJointNum + 1);
    
    CNumericalIkSolver::m_dCostTolerance = 0.00001;
    CNumericalIkSolver::m_dConstraintTolerance = 0.005;
    CNumericalIkSolver::m_iIterationTolerance = 300;
    
    m_iInitFlag = SUCCESS;
    
    return SUCCESS;
    
}

int CNumericalIkSolver::UnInitRobot()
{
    assert(m_iInitFlag == 0);
    
    vector <Joint>().swap(m_vRobotArm);
    m_iInitFlag = FAIL;
    
    return SUCCESS;
}

int CNumericalIkSolver::CQuadCostFunctor::SetAngles(VectorXd MatAngles) const
{
    assert(m_iInitFlag);
    int iRSize = m_vRobotArm.size();//
    for (int i = 0; i < iRSize; i++)
    {
        m_vRobotArm[i].dAngle = MatAngles(i);
    }
    return SUCCESS;
}

int CNumericalIkSolver::CQuadCostFunctor::ForwardKinematics(VectorXd MatAngles) const
{
    assert(m_iInitFlag == 0);
    
    SetAngles(MatAngles);
    
    int iSize = m_vRobotArm.size();
    int iEnd = iSize - 1;
    Vector3d MatTempOffset;
    
    for (int i = 1; i < iSize; i++)
    {
        int iTempParent = m_vRobotArm[i].iParent - 1;
        Matrix3d MatTempRotation = m_vRobotArm[i].MatRotation;
        Vector3d MatTempAxis = m_vRobotArm[i].MatAxis;
        double dTempAngle = m_vRobotArm[i].dAngle;
        
        GenerateRotationMatrix(MatTempRotation, MatTempAxis, dTempAngle);
        
        m_vRobotArm[i].MatRotation = MatTempRotation;
        
        m_vRobotArm[i].MatOrientation = m_vRobotArm[iTempParent].MatOrientation * m_vRobotArm[i].MatRotation;

        MatTempOffset = m_vRobotArm[iTempParent].MatOrientation * m_vRobotArm[i].MatOffset;
        m_vRobotArm[i].MatPosition = m_vRobotArm[iTempParent].MatPosition + MatTempOffset;
    }
    
    m_MatElbow = m_vRobotArm[iEnd].MatPosition;
    MatTempOffset = m_vRobotArm[iEnd].MatOrientation * m_MatEndEffOffset;//'
    m_MatEndEffector = m_MatElbow + MatTempOffset;//no need to transpose
    m_vRobotArm[iEnd].MatEndEffector = m_MatEndEffector;
    m_vRobotArm[iEnd].MatElbow = m_MatElbow;
    
    return SUCCESS;
//    cout<<"finished fk"<<endl;
}

int CNumericalIkSolver::CQuadCostFunctor::GenerateRotationMatrix(Matrix3d & MatRotation, Vector3d MatAxis, double dAngle) const
{
    assert(m_iInitFlag == 0);
    
    double dC = cos(dAngle);
    double dS = sin(dAngle);
    double dT = 1 - dC;
    
    double dAxisLen = 0;
    
    for (int i = 0; i < 3; i++)
    {
        dAxisLen += MatAxis(i) * MatAxis(i);
    }
    
    MatAxis = MatAxis / sqrt(dAxisLen);
    
    double dX = MatAxis(0);
    double dY = MatAxis(1);
    double dZ = MatAxis(2);
    
    MatRotation << dT * dX * dX + dC, dT * dX * dY - dZ * dS, dT * dX * dZ + dY * dS,
                dT * dX * dY + dZ * dS, dT * dY * dY + dC, dT * dY * dZ - dX * dS,
                dT * dX * dZ - dY * dS, dT * dY * dZ + dX * dS, dT * dZ * dZ + dC;

    return SUCCESS;
}

int CNumericalIkSolver::Normalization()
{
    assert(m_iInitFlag == 0);
    
    int iSize = m_vRobotArm.size() - 1;
    
    for (int i = 0; i < iSize; i++){
        if (m_dAngles[i] < -2*m_dPI || m_dAngles[i] > 2*m_dPI) // normalization
            m_dAngles[i] = m_dAngles[i] - (double)((int)(m_dAngles[i]/(2*m_dPI)))*2*m_dPI;
        
        if (m_dAngles[i] > m_dPI)
            m_dAngles[i] -= 2*m_dPI;
        if (m_dAngles[i] < -m_dPI)
            m_dAngles[i] += 2*m_dPI;
        
    }
    return SUCCESS;
}


int CNumericalIkSolver::RunSolver(Vector3d MatTEndEffector, Vector3d MatTElbow, double dMainW, double dCoupledW, MyConstraintPtr pConstraint, MyObjectivePtr pObjective)
{
    assert(m_iInitFlag == 0);//check whether the robot model is already initiated
    
    if (m_iInitFlag == 0)
        return FAIL;
    
    Obj = pObjective;
    Constraint = pConstraint;
    
    m_MatTargetEndEffector = MatTEndEffector;
    m_MatTargetElbow = MatTElbow;
    
    
    
    m_bFlag = false;
    
    srand((int)time(0));
    
    int iIterations = 0;
//    int iResultFlag = 1;
//    int const iRSize = m_vRobotArm.size() - 1;
    
    while(iIterations < m_iIterationTolerance)
    {
        cout<<iIterations<<endl;
        for (int i = 0; i < m_iJointNum; i++) //initial state randomness, can be modified more efficiently
        {
            m_dAngles[i] = m_vRobotArm[i+1].dLowBound + (m_vRobotArm[i+1].dUpBound - m_vRobotArm[i+1].dLowBound) * ((double) rand() / (RAND_MAX));
        }
        
        m_dAngles[m_iJointNum] = pObjective(m_vRobotArm, m_MatTargetEndEffector);//constraint
    //      Prob init
        Problem problem;
    
    //      Cost func init
    
    //NumericDiffCostFunction <QuadCostFunc,   FORWARD,               1,           JOINT_NUM>
    //                              |             |                   |                |
    //                         CostFunction       |                   |                |
    //                                     for speed 1st, other       |                |
    //                                        options are:       dimension of          |
    //                                       CENTRAL, RIDDLES     residuals       dimension of
    //                                                                             input params
    
    //NumericDiffCostFunction: other choices are AutoDiffCostFunction, AnalysisDiffCostFunction, haven't tried so far.
    //set dimension of residuals to 2 to add elbow cost, doesn't work well so far
        DynamicNumericDiffCostFunction <CQuadCostFunctor,FORWARD> * cost_function1 = new DynamicNumericDiffCostFunction <CQuadCostFunctor,FORWARD>( new CQuadCostFunctor(dMainW, dCoupledW, MatTEndEffector, MatTElbow));
        
        //<CQuadCostFunctor, FORWARD, 1, 6>
        
        cost_function1 -> AddParameterBlock(m_iJointNum+1);
//        cost_function1 -> AddParameterBlock(1);
        cost_function1 -> SetNumResiduals(2);
        
        
    
        //m_iJointNum
    
    //      Residual adding
        problem.AddResidualBlock(cost_function1, NULL, m_dAngles);
    //        problem.AddResidualBlock(cost_function2, NULL, inAngleNoEff);
    
    //      Param boundary setting, temporarily useless, replaced by func Satisfied() called inside the costfunc
        
        for (int j = 0; j < m_iJointNum; j++)
        {
            problem.SetParameterLowerBound(m_dAngles,j,m_vRobotArm[j+1].dLowBound);
            problem.SetParameterUpperBound(m_dAngles,j,m_vRobotArm[j+1].dUpBound);
        }
        
        problem.SetParameterUpperBound(m_dAngles, m_iJointNum, m_dConstraintTolerance);//constraint
        
        
        
    //      options setting
        Solver::Options options;
    
    //      options.max_num_iterations = 5;
    //        options.linear_solver_type = ceres::DENSE_QR;
    //        options.minimizer_progress_to_stdout = true;
    //      options.function_tolerance = 1e-4;
        options.max_num_iterations = 300; // default 50, not enough here
        options.update_state_every_iteration = true;
    //        options.minimizer_type = ceres::LINE_SEARCH; //doesn't work!!!
    //        options.line_search_direction_type = ceres::BFGS; //default LBFGS
    
    //      Summary setting
        Solver::Summary summary;
    
    //      Solver process
        Solve(options, &problem, &summary);
    
        iIterations++;
        
        
//        vector<Joint> vTempRobot;
//        
//        GetRobotParams(vTempRobot);
        
//        cout<< "!!!!!!"<<pConstraint(m_vRobotArm)<<endl;
        
        double dConstraintError;
        
        if (pConstraint != NULL)
        {
            dConstraintError = pConstraint(m_vRobotArm, m_MatTargetEndEffector);
        }
        else
        {
            dConstraintError = DefaultConstraint();
        }
    
        if ( m_dAngles[m_iJointNum] - Constraint(m_vRobotArm, m_MatTargetEndEffector) >= -0.001 ) //dConstraintError
        {
            Normalization();

            return SUCCESS;
        }

//        delete cost_function1;
    }
    
//    delete m_dAngles;
    
    return FAIL;
}

int CNumericalIkSolver::PrintResult()
{
    assert(m_iInitFlag == 0);
    
    cout <<"-> 0.001 "<<m_dAngles[0] <<" "<<m_dAngles[1]<<" "<<m_dAngles[2]<<" "<<m_dAngles[3]<<" "<<m_dAngles[4]<<" "<<m_dAngles[5]<<" "<<endl;
    cout <<"endeff: "<<m_MatEndEffector.transpose()<<endl;
    cout <<"cost: "<<m_dCost<<endl;;
    return SUCCESS;
}

/*
struct Joint // joint information
{
    int iIndex;
    int iParent;
    double dAngle;
    double dLowBound;
    double dUpBound;
    
    std::string strName;
    
    Eigen::Vector3d MatAxis, MatOffset, MatPosition, MatEndEffector, MatElbow;
    Eigen::Matrix3d MatOrientation,MatRotation;
    
};
*/

int CNumericalIkSolver::GetRobotParams(vector<Joint> & vRobot)
{
    Joint * sJoint;
    sJoint = (Joint*)malloc(sizeof(Joint)*1);
    for (int i = 0; i <= m_iJointNum; i++)
    {
        sJoint->iIndex = m_vRobotArm[i].iIndex;
        sJoint->iParent = m_vRobotArm[i].iParent;
        sJoint->dAngle = m_vRobotArm[i].dAngle;
        sJoint->dLowBound = m_vRobotArm[i].dLowBound;
        sJoint->dUpBound = m_vRobotArm[i].dUpBound;
        sJoint->strName = m_vRobotArm[i].strName;
        sJoint->MatAxis = m_vRobotArm[i].MatAxis;
        sJoint->MatOffset = m_vRobotArm[i].MatOffset;
        sJoint->MatPosition = m_vRobotArm[i].MatPosition;
        sJoint->MatEndEffector = m_vRobotArm[i].MatEndEffector;
        sJoint->MatElbow = m_vRobotArm[i].MatElbow;
        sJoint->MatOrientation = m_vRobotArm[i].MatOrientation;
        sJoint->MatRotation = m_vRobotArm[i].MatRotation;
        
        vRobot.push_back(*sJoint);
    }
    
    if(sJoint)
        free(sJoint);
    return 0;
}

