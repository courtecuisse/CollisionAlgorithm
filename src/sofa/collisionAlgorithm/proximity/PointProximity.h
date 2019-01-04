#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/element/PointElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointProximity : public ConstraintProximity
{
public :
    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    PointProximity(const PointElement * elmt)
    : m_element(elmt)
    , m_state(elmt->geometry()->getState()) {}

    defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        return pos[m_element->m_pid];
    }

    defaulttype::Vector3 getNormal() const
    {
        return defaulttype::Vector3(1,0,0);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, ConstraintNormal & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[m_state].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);

            c_it.addCol(m_element->m_pid, normals[j] * fact);
        }

        c1_d.endEdit();
    }

//    sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() { return m_state; }

protected:
    const PointElement* m_element;
    sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * m_state;
};

}

}
