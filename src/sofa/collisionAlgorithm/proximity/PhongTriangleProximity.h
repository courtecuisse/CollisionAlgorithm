#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class PhongTriangleProximity : public TriangleProximity<GEOMETRY> {
public :

    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    PhongTriangleProximity(const GEOMETRY * geo,unsigned tid, unsigned p1,unsigned p2,unsigned p3,double f1,double f2,double f3)
    : TriangleProximity<GEOMETRY>(geo,tid, p1,p2,p3,f1,f2,f3) {}

    inline defaulttype::Vector3 getNormal() const {
        return this->m_geometry->pointNormals()[this->m_pid[0]] * this->m_fact[0] +
               this->m_geometry->pointNormals()[this->m_pid[1]] * this->m_fact[1] +
               this->m_geometry->pointNormals()[this->m_pid[2]] * this->m_fact[2];
    }

};

}

}
