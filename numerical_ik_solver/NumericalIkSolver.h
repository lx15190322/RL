//
//  NumericalIkSolver.h
//  
//
//  Created by Xuan Liu on 17/6/23.
//
//

// 

#ifndef C_NUMERICAL_IK_SOLVER_H
#define C_NUMERICAL_IK_SOLVER_H

#include <vector>
#include <iostream>
#include <string>
//#include <cmath>

#include "Eigen/Dense"
//#include "ceres/ceres.h"
//#include "gflags/gflags.h"
//#include "glog/glog.h"

#define JOINT_NUM 6
#define SUCCESS 1
#define FAIL 0
#define LEFT 1
#define RIGHT 0


//using ceres::NumericDiffCostFunction;
//using ceres::AutoDiffCostFunction;
//using ceres::CENTRAL;
//using ceres::FORWARD;
//using ceres::CostFunction;
//using ceres::Problem;
//using ceres::Solver;
//using ceres::Solve;


// struct joint
// struct result{ angles, resultType}
// enum {success, timeout, no_solution}
// 

//**should i open some important ceres params for the users?

//calling method
// 1. CNumericalIkSolver iCNumericalIkSolver; 
// 2. iCNumericalIkSolver.init()
// 3. result = iCNumericalIkSolver.RunSolver()


//struct Joint // joint information
//{
//    int iIndex;
//    int iParent;
//    double dAngle;
//    double dLowBound;
//    double dUpBound;
//    
//    std::string strName;
//    
//    Eigen::Vector3d MatAxis, MatOffset, MatPosition, MatEndEffector, MatElbow;
//    Eigen::Matrix3d MatOrientation,MatRotation;
//    
//};

//extern bool m_bFlag;

//extern int m_iInitFlag;

//extern std::vector<Joint> m_vRobotArm;
//extern Eigen::Vector3d m_MatEndEffector;
//extern Eigen::Vector3d m_MatElbow;
//extern Eigen::Vector3d m_MatEndEffOffset;
//extern double m_dCost;
//extern double m_dCoupledCost;
//extern double m_dConstraint;
//extern double * m_dAngles;

class CNumericalIkSolver
{
    public:
 
    CNumericalIkSolver()
    {
        m_iInitFlag = FAIL;
    }
    ~CNumericalIkSolver() {};
    
    
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

    
    //TODO add a loss function, or maybe loss functor class operator, design a kind of loss functor
    
    typedef double(*MyObjectivePtr)(std::vector<CNumericalIkSolver::Joint>, Eigen::MatrixXd);
    
    static MyObjectivePtr Obj;
    
    typedef double(*MyConstraintPtr)(std::vector<CNumericalIkSolver::Joint>, Eigen::MatrixXd);
    
    static MyConstraintPtr Constraint;

	class CQuadCostFunctor
    {
        public:
        
        CQuadCostFunctor(double dW1, double dW2, Eigen::Vector3d MatTEff, Eigen::Vector3d MatTElbow) :
        dW1_(dW1), dW2_(dW2), MatTEff_(MatTEff), MatTElbow_(MatTElbow) {}
        ~CQuadCostFunctor() {}
        
        int Satisfied(const double * const dAngles) const;
        int SetAngles(Eigen::VectorXd MatAngles) const; // std::vector<Joint> &vRobot
        int ForwardKinematics(Eigen::VectorXd MatAngles) const;//std::vector<Joint> &vRobot
        int GenerateRotationMatrix(Eigen::Matrix3d &MatRotation, Eigen::Vector3d MatAxis, double dAngle) const;
        
        template <typename T>
        bool operator()(T const * const * params, T * residuals) const
        {
            int iRSize = m_vRobotArm.size() - 1;
            Eigen::VectorXd MatAngles(iRSize + 1);
            Eigen::Vector3d MatDiffEnd; // error between endeffector state and target endeff state
            Eigen::Vector3d MatDiffElbow; // error between elbow state and target elbow state
            
            MatAngles(0) = 0.001;
            // first angle is the base angle, not included in the calculation, can be set as any number
            
            for (int i = 1; i <= iRSize; i++) //Generating Eigen vector
            {
                MatAngles(i) = params[0][i-1];
            }

            ForwardKinematics(MatAngles); //Calling FK, hidden function doesn't matter when doing numerical diff, but won't work when doing analytical optimization.
            
            MatDiffEnd = MatTEff_ - m_MatEndEffector;
            MatDiffElbow = MatTElbow_ - m_MatElbow;
            
            m_dCost = MatDiffEnd.dot(MatDiffEnd.transpose());
//            m_dCoupledCost = MatDiffElbow.dot(MatDiffElbow.transpose());
//            residuals[0] = dW1_ * m_dCost;
            
            residuals[0] = dW1_ * Obj(m_vRobotArm, MatTEff_);
            residuals[1] = dW2_ * (params[0][m_iJointNum] - Constraint(m_vRobotArm, MatTEff_));
//            residuals[1] = dW2_ * m_dCoupledCost;
            
//            std::cout<<residuals[0]<<"!!"<<residuals[1]<<std::endl;
//            m_bFlag = Satisfied(params);
            m_bFlag = true;
            
            return m_bFlag;

        }
        
        private:
        
        const double dW1_, dW2_;
        const Eigen::Vector3d MatTEff_, MatTElbow_;
    };
    
    //    int Satisfied(const double * const dAngles);
    //    int SetAngles(Eigen::VectorXd MatAngles); // std::vector<Joint> &vRobot
    //    int ForwardKinematics(Eigen::VectorXd MatAngles);//std::vector<Joint> &vRobot
    //    int GenerateRotationMatrix(Eigen::Matrix3d &MatRotation, Eigen::Vector3d MatAxis, double dAngle);
    
    int InitRobot(int iArmFlag = LEFT); // std::vector<Joint> &vRobot
    int UnInitRobot(); // std::vector<Joint> &vRobot
    int Normalization();
    int PrintResult();
    
    //TODO add a complete constraint checking function
    // Constraint();

//	class TerminationCallBack

    double DefaultConstraint();
//    virtual int MyConstraint();
    
    
    //important constraint
    
//    virtual int myConstraint();
//    void AddConstraintBlock();
//    typedef double(*MyConstraintPtr)(std::vector<CNumericalIkSolver::Joint>, Eigen::MatrixXd);
    
    
    int RunSolver(Eigen::Vector3d MatTEndEffector, Eigen::Vector3d MatTElbow, double dMainW, double dCoupledW, MyConstraintPtr pConstraint = NULL, MyObjectivePtr pObjective = NULL); // eint //const Eigen::Vector3d MatTEndEffector, const Eigen::Vector3d MatTElbow // T &inAngle
    
    int GetRobotParams(std::vector<Joint> & vRobot);
    
    private:
    //robot
//    std::vector<Joint> m_vRobotArm;
//    double * m_dAngles;
    
//    Eigen::Vector3d m_MatEndEffOffset;
//    Eigen::Vector3d m_MatEndEffector;
//    Eigen::Vector3d m_MatElbow;
    
//    int m_iInitFlag;
//    int m_iJointNum;
    
    
    //option_settings for ceres optimizer
    //TODO: add options for ceres params
    //TODO: add options for costFunction
    //TODO: add options for constraint
        //current constraints we might probably have: reaching constraint (tolerance distance from the desired position), self collision, viapoint constraint, joint direction constraint, angular contribution/weight constraint
    
//    int m_iCostDimension;
    
    //option_settings for iksolver
    static int m_iJointNum;
    static int m_iIterationTolerance;
    static int m_iInitFlag;
    static int m_iArmFlag;
    
    static double m_dCostTolerance;
    static double m_dConstraintTolerance;
    static double m_dCost;
    
    
    static bool m_bFlag;
    static double * m_dAngles;
    
    static Eigen::Vector3d m_MatEndEffector;
    static Eigen::Vector3d m_MatElbow;
    static Eigen::Vector3d m_MatEndEffOffset;
    
    static std::vector<CNumericalIkSolver::Joint> m_vRobotArm;
//    static std::vector<CNumericalIkSolver::Joint> m_vRightArm;
    
    static const double m_dPI;
    
    
    Eigen::Vector3d m_MatTargetEndEffector;
    Eigen::Vector3d m_MatTargetElbow;
    // double * m_dRobotAngleState;
    
    
//    double m_dCost;
//    double m_dCoupledCost;
//    double m_dConstraint;
    
//    bool m_bFlag;
    
	
};

//typedef int(*MyConstraintPtr)(std::vector<CNumericalIkSolver::Joint>);

#endif //C_NUMERICAL_IK_SOLVER_H


