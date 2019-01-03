#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TriangleProximity : public ConstraintProximity
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

    static ConstraintProximity::SPtr createProximity(const TriangleElement * elmt,double f1,double f2,double f3);

    TriangleProximity(const TriangleElement * elmt,double f1,double f2,double f3)
    : m_element(elmt)
    , m_state(elmt->geometry()->getState()) {
        m_fact[0] = f1;
        m_fact[1] = f2;
        m_fact[2] = f3;
    }

    virtual ~TriangleProximity()
    {
    }

    virtual defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);

        return pos[m_element->m_pid[0]] * m_fact[0] +
               pos[m_element->m_pid[1]] * m_fact[1] +
               pos[m_element->m_pid[2]] * m_fact[2];
    }

    virtual defaulttype::Vector3 getNormal() const
    {
        return m_element->geometry()->getNormal(m_element->id(), m_fact);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, ConstraintNormal & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[m_state].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);

            c_it.addCol(m_element->m_pid[0], normals[j] * m_fact[0] * fact);
            c_it.addCol(m_element->m_pid[1], normals[j] * m_fact[1] * fact);
            c_it.addCol(m_element->m_pid[2], normals[j] * m_fact[2] * fact);
        }

        c1_d.endEdit();
    }

    double m_fact[3];
protected:
    const TriangleElement* m_element;
    sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * m_state;
};

}

}
