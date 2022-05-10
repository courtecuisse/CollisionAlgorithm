#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class FixedProximity : public BaseProximity{
public:
  typedef std::shared_ptr<FixedProximity> SPtr;
  typedef typename collisionAlgorithm::BaseProximity Inherits;
  typedef type::Vec3d Vec3d;

  FixedProximity(Vec3d P ) : m_position(P) { m_normal = Vec3d(); }
  FixedProximity(Vec3d P, Vec3d N ) : m_position(P), m_normal(N) {}

  /// return proximity position in a vector3
  virtual Vec3d getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
    return m_position;
  }

  /// return normal in a vector3
  virtual Vec3d getNormal() const {
    return m_normal;
  }

  virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const {}

  virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const {}


private:
  Vec3d m_position;
  Vec3d m_normal;

};

}

