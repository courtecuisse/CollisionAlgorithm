#ifndef SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H
#define SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H

#include "CollisionAlgorithm.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>

#include <sofa/core/behavior/ForceField.inl>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <sofa/defaulttype/VecTypes.h>
#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/PointSetTopologyModifier.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>

#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/SolidTypes.h>

namespace sofa {

namespace core {

namespace behavior {


void CollisionAlgorithm::PCAPointCloud(helper::vector<defaulttype::Vector3> & pos, defaulttype::Vector3 & C,defaulttype::Quat & R1, defaulttype::Quat & R2, defaulttype::Quat & R3, defaulttype::Quat & R4) {

    C = defaulttype::Vector3(0,0,0);
    for (unsigned i = 0; i<pos.size();i++) C += pos[i];
    C /= pos.size();

    Eigen::Matrix3d Cov = Eigen::MatrixXd::Zero(3,3);
    for (unsigned k=0;k<pos.size();k++) {
        defaulttype::Vector3 P = pos[k] - C;

        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                double cov = P[j] * P[i];
                Cov(j,i) += cov;
            }
        }
    }
    Cov *= 1.0/(double) (pos.size());
    ////Transform with pseudoEigen
    Eigen::EigenSolver<Eigen::Matrix3d> solver(Cov);
    Eigen::Matrix3d s_vec = solver.pseudoEigenvectors();
    Eigen::Matrix3d s_val = solver.pseudoEigenvalueMatrix();

    defaulttype::Vector3 e_val(s_val(0,0),s_val(1,1),s_val(2,2));
    defaulttype::Vector3 e_vec[3] = {defaulttype::Vector3(s_vec(0,0)*e_val[0],s_vec(1,0)*e_val[0],s_vec(2,0)*e_val[0]),
                        defaulttype::Vector3(s_vec(0,1)*e_val[1],s_vec(1,1)*e_val[1],s_vec(2,1)*e_val[1]),
                        defaulttype::Vector3(s_vec(0,2)*e_val[2],s_vec(1,2)*e_val[2],s_vec(2,2)*e_val[2])};

    //sort according to the eigen values
    defaulttype::Vec3i order;
    if (e_val[1]<e_val[2]) {
        if (e_val[0]<e_val[1]) order = defaulttype::Vec3i(0,1,2);
        else if (e_val[0]<e_val[2]) order = defaulttype::Vec3i(1,0,2);
        else order = defaulttype::Vec3i(1,2,0);
    } else {
        if (e_val[0]<e_val[2]) order = defaulttype::Vec3i(0,2,1);
        else if (e_val[0]<e_val[1]) order = defaulttype::Vec3i(2,0,1);
        else order = defaulttype::Vec3i(2,1,0);
    }


    defaulttype::Vector3 rX = e_vec[order[2]];
    defaulttype::Vector3 rY = e_vec[order[1]];
    defaulttype::Vector3 rZ = defaulttype::cross(rX,rY);

    rX.normalize();
    rY.normalize();
    rZ.normalize();

    R1.fromFrame(rX,rY,rZ);
    R1.normalize();

    rY = - rY;
    rZ = - rZ;

    R2.fromFrame(rX,rY,rZ);
    R2.normalize();

    rX = - rX;
    rY = - rY;

    R3.fromFrame(rX,rY,rZ);
    R3.normalize();

    rY = - rY;
    rZ = - rZ;

    R4.fromFrame(rX,rY,rZ);
    R4.normalize();
// /////////////////////////////////////////////

//    C = defaulttype::Vector3(0,0,0);
//    for (unsigned i = 0; i<pos.size();i++) C += pos[i];
//    C /= pos.size();

//    //                                      i
//    //                                      |
//    //                                      v
//    //                                 Q1x Q1y Q1z
//    //                            k->  Q2x Q2y Q2z
//    //                                 Q3x Q3y Q3z
//    //                                 Q4x Q4y Q4z
//    //                                 Q5x Q5y Q5z
//    //            k                    Q6x Q6y Q6z
//    //            |                    Q7x Q7y Q7z
//    //            v
//    //       P1x P2x P3x P4x P5x P6x
//    // j->   P1y P2y P3y P4y P5y P6y
//    //       P1z p2z P3z P4z P5z p6z

//    Eigen::Matrix3d Cov = Eigen::MatrixXd::Zero(3,3);
//    for (unsigned k=0;k<pos.size();k++) {
//        defaulttype::Vector3 P = pos[k] - C;

//        for (unsigned j=0;j<3;j++) {
//            for (unsigned i=0;i<3;i++) {
//                double cov = P[j] * P[i];
//                Cov(j,i) += cov;
//            }
//        }
//    }
//    Cov *= 1.0/(double) (pos.size());

//    ////Transform with SVD
//    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    Eigen::Matrix3d Rot = Eigen::MatrixXd::Zero(3,3);
//    Eigen::Matrix3d S   = Eigen::MatrixXd::Zero(3,3);
//    S(0,0) = 1; S(1,1) = 1; S(2,2) = 1;
//    if((svd.matrixU()).determinant()<0) S(2,2) = -1;

//    //Test to find out the matrix S of Umeyama formula
//    Eigen::JacobiSVD<Eigen::Matrix3d> svd_U((svd.matrixU()), Eigen::ComputeFullU | Eigen::ComputeFullV);
//    const Eigen::Vector3d& d = svd_U.singularValues();
//    int rank_U = 0; for (int i=0; i<3; ++i) if (!Eigen::internal::isMuchSmallerThan(d.coeff(i),d.coeff(0))) ++rank_U;

//    if(rank_U==2){
//        if((svd_U.matrixU().determinant()*svd_U.matrixV().determinant()) > 0){
//            Rot = svd.matrixU();
//        }else{
//            double s = S(2,2); S(2,2) = -1;
//            Rot = svd.matrixU()*S;
//            S(2,2) = s;
//        }
//    }else{
//        Rot = svd.matrixU()*S;
//    }

//    defaulttype::Matrix3 M;
//    for (unsigned j=0;j<3;j++)
//        for (unsigned i=0;i<3;i++)
//            M[j][i] = Rot(j,i);

//    R.fromMatrix(M);

}
void CollisionAlgorithm::PCAPointCloud(helper::vector<defaulttype::Vector3> & pos, defaulttype::Vector3 & C,defaulttype::Quat & R1) {

    C = defaulttype::Vector3(0,0,0);
    for (unsigned i = 0; i<pos.size();i++) C += pos[i];
    C /= pos.size();

    Eigen::Matrix3d Cov = Eigen::MatrixXd::Zero(3,3);
    for (unsigned k=0;k<pos.size();k++) {
        defaulttype::Vector3 P = pos[k] - C;

        for (unsigned j=0;j<3;j++) {
            for (unsigned i=0;i<3;i++) {
                double cov = P[j] * P[i];
                Cov(j,i) += cov;
            }
        }
    }
    Cov *= 1.0/(double) (pos.size());
    ////Transform with pseudoEigen
    Eigen::EigenSolver<Eigen::Matrix3d> solver(Cov);
    Eigen::Matrix3d s_vec = solver.pseudoEigenvectors();
    Eigen::Matrix3d s_val = solver.pseudoEigenvalueMatrix();

    defaulttype::Vector3 e_val(s_val(0,0),s_val(1,1),s_val(2,2));
    defaulttype::Vector3 e_vec[3] = {defaulttype::Vector3(s_vec(0,0)*e_val[0],s_vec(1,0)*e_val[0],s_vec(2,0)*e_val[0]),
                        defaulttype::Vector3(s_vec(0,1)*e_val[1],s_vec(1,1)*e_val[1],s_vec(2,1)*e_val[1]),
                        defaulttype::Vector3(s_vec(0,2)*e_val[2],s_vec(1,2)*e_val[2],s_vec(2,2)*e_val[2])};

    //sort according to the eigen values
    defaulttype::Vec3i order;
    if (e_val[1]<e_val[2]) {
        if (e_val[0]<e_val[1]) order = defaulttype::Vec3i(0,1,2);
        else if (e_val[0]<e_val[2]) order = defaulttype::Vec3i(1,0,2);
        else order = defaulttype::Vec3i(1,2,0);
    } else {
        if (e_val[0]<e_val[2]) order = defaulttype::Vec3i(0,2,1);
        else if (e_val[0]<e_val[1]) order = defaulttype::Vec3i(2,0,1);
        else order = defaulttype::Vec3i(2,1,0);
    }


    defaulttype::Vector3 rX = e_vec[order[2]];
    defaulttype::Vector3 rY = e_vec[order[1]];
    defaulttype::Vector3 rZ = defaulttype::cross(rX,rY);

    rX.normalize();
    rY.normalize();
    rZ.normalize();

    R1.fromFrame(rX,rY,rZ);
    R1.normalize();
// /////////////////////////////////////////////

//    C = defaulttype::Vector3(0,0,0);
//    for (unsigned i = 0; i<pos.size();i++) C += pos[i];
//    C /= pos.size();

//    //                                      i
//    //                                      |
//    //                                      v
//    //                                 Q1x Q1y Q1z
//    //                            k->  Q2x Q2y Q2z
//    //                                 Q3x Q3y Q3z
//    //                                 Q4x Q4y Q4z
//    //                                 Q5x Q5y Q5z
//    //            k                    Q6x Q6y Q6z
//    //            |                    Q7x Q7y Q7z
//    //            v
//    //       P1x P2x P3x P4x P5x P6x
//    // j->   P1y P2y P3y P4y P5y P6y
//    //       P1z p2z P3z P4z P5z p6z

//    Eigen::Matrix3d Cov = Eigen::MatrixXd::Zero(3,3);
//    for (unsigned k=0;k<pos.size();k++) {
//        defaulttype::Vector3 P = pos[k] - C;

//        for (unsigned j=0;j<3;j++) {
//            for (unsigned i=0;i<3;i++) {
//                double cov = P[j] * P[i];
//                Cov(j,i) += cov;
//            }
//        }
//    }
//    Cov *= 1.0/(double) (pos.size());

//    ////Transform with SVD
//    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    Eigen::Matrix3d Rot = Eigen::MatrixXd::Zero(3,3);
//    Eigen::Matrix3d S   = Eigen::MatrixXd::Zero(3,3);
//    S(0,0) = 1; S(1,1) = 1; S(2,2) = 1;
//    if((svd.matrixU()).determinant()<0) S(2,2) = -1;

//    //Test to find out the matrix S of Umeyama formula
//    Eigen::JacobiSVD<Eigen::Matrix3d> svd_U((svd.matrixU()), Eigen::ComputeFullU | Eigen::ComputeFullV);
//    const Eigen::Vector3d& d = svd_U.singularValues();
//    int rank_U = 0; for (int i=0; i<3; ++i) if (!Eigen::internal::isMuchSmallerThan(d.coeff(i),d.coeff(0))) ++rank_U;

//    if(rank_U==2){
//        if((svd_U.matrixU().determinant()*svd_U.matrixV().determinant()) > 0){
//            Rot = svd.matrixU();
//        }else{
//            double s = S(2,2); S(2,2) = -1;
//            Rot = svd.matrixU()*S;
//            S(2,2) = s;
//        }
//    }else{
//        Rot = svd.matrixU()*S;
//    }

//    defaulttype::Matrix3 M;
//    for (unsigned j=0;j<3;j++)
//        for (unsigned i=0;i<3;i++)
//            M[j][i] = Rot(j,i);

//    R.fromMatrix(M);

}
double CollisionAlgorithm::Score(helper::vector<defaulttype::Vector3> & p_from,helper::vector<defaulttype::Vector3> & p_dst,helper::vector<int> & bindId) {
    double score =0;
    for (unsigned p=0;p<p_from.size();p++) {
        if (bindId[p]==-1) continue;
        defaulttype::Vector3 P=p_from[p];
        defaulttype::Vector3 Q=p_dst[bindId[p]];
        score=score +(P-Q).norm();
    }
    return score;
}
} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H

