#pragma once

#include <memory>
#include <map>
#include <vector>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/VecId.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/MultiVecId.h>

namespace sofa
{

namespace collisionAlgorithm
{

class ConstraintElement;
class BaseGeometry;

class ConstraintProximity {
public :
    typedef std::shared_ptr<ConstraintProximity> SPtr;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    class ConstraintNormal {
    public:
        unsigned size() const {
            return m_normals.size();
        }

        ConstraintNormal() {}

        ConstraintNormal(defaulttype::Vector3 n1) {
            m_normals.push_back(n1.normalized());
        }

        ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2) {
            m_normals.push_back(n1.normalized());
            m_normals.push_back(n2.normalized());
        }

        ConstraintNormal(defaulttype::Vector3 n1,defaulttype::Vector3 n2,defaulttype::Vector3 n3) {
            m_normals.push_back(n1.normalized());
            m_normals.push_back(n2.normalized());
            m_normals.push_back(n3.normalized());
        }

        void normalize(unsigned sz) {
            if (m_normals.size() > sz) m_normals.resize(sz);
        }

        static ConstraintNormal createFrame(defaulttype::Vector3 N1 = defaulttype::Vector3()) {
            if (N1.norm() == 0) N1 = defaulttype::Vector3(1,0,0);
            defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(0,1,0)))>0.99) ? defaulttype::Vector3(0,0,1) : defaulttype::Vector3(0,1,0)));
            defaulttype::Vector3 N3 = cross(N1,N2);

            return ConstraintNormal(N1,N2,N3);
        }

        defaulttype::Vector3 operator[](unsigned sz) {
            return m_normals[sz];
        }

        std::vector<defaulttype::Vector3> m_normals;
        collisionAlgorithm::ConstraintProximity::SPtr m_prox;
    };

    virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    virtual defaulttype::Vector3 getNormal() const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, ConstraintNormal & normals, double fact, unsigned constraintId) const = 0;

//    virtual sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() = 0;

//    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda, unsigned constraintId) const = 0;
};

class FixedProximity : public ConstraintProximity {
public:

    FixedProximity(defaulttype::Vector3 & p) : m_position(p) {}

    const ConstraintElement* element() const {
        return NULL;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId ) const {
        return m_position;
    }

    virtual defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3();
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId /*cId*/, ConstraintNormal & /*m_normals*/, double /*fact*/, unsigned /*constraintId*/) const {}

//    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId /*res*/, const sofa::defaulttype::BaseVector* /*lambda*/, unsigned /*constraintId*/) const {}

//    virtual sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() { return NULL; }

    defaulttype::Vector3 m_position;
};

}

using ConstraintNormal=collisionAlgorithm::ConstraintProximity::ConstraintNormal;

}
