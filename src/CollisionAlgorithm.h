#ifndef SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H
#define SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include "ConstraintGeometry.h"
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

class CollisionAlgorithm {
public:

    static ConstraintProximity findClosestProximity(const defaulttype::Vector3 & T, BaseGeometry * geo);

    static void BindPointCloud(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist = 0.0);
    static void KdTreeClosestPoint(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist = 0.0);
    static void totalDistanceBind(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist = 0.0);
    static void kdtreeBind(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist = 0.0);
    static void BindPointCloudReinit(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist);
    static void HybridBind(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double coeff, double minDist, double minScore);
    static void PCAPointCloud(helper::vector<defaulttype::Vector3> &pos, defaulttype::Vector3 & C, defaulttype::Quat & R1, defaulttype::Quat &R2, defaulttype::Quat &R3, defaulttype::Quat &R4);
    static void PCAPointCloud(helper::vector<defaulttype::Vector3> & pos, defaulttype::Vector3 & C,defaulttype::Quat & R1);
    static double Score(helper::vector<defaulttype::Vector3> & p_dst, helper::vector<defaulttype::Vector3> & p_from, helper::vector<int> & bindId);
    static void ManualBind(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist = 0.0);
};
} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
