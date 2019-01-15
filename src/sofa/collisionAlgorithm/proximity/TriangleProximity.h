#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class TriangleProximity : public TBaseProximity<DataTypes> {
    friend class TriangleGeometry<DataTypes>;

public :
    typedef std::shared_ptr<TriangleProximity> SPtr;
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

    TriangleProximity(unsigned eid,unsigned p1,unsigned p2,unsigned p3,double f1,double f2,double f3, const TriangleGeometry<DataTypes> * geometry, State * state)
    : TBaseProximity<DataTypes>(state)
    , m_eid(eid)
    , m_geometry(geometry) {
        m_pid[0] = p1;
        m_pid[1] = p2;
        m_pid[2] = p3;

        m_fact[0] = f1;
        m_fact[1] = f2;
        m_fact[2] = f3;
    }

    virtual defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_state->read(v);

        return pos[m_pid[0]] * m_fact[0] +
               pos[m_pid[1]] * m_fact[1] +
               pos[m_pid[2]] * m_fact[2];
    }

    virtual defaulttype::Vector3 getNormal() const {
        return m_geometry->getNormal(this);
    }

    void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_pid[0], N * m_fact[0]);
        it.addCol(m_pid[1], N * m_fact[1]);
        it.addCol(m_pid[2], N * m_fact[2]);
    }

    const double * getFactors() const { return m_fact; }

    const unsigned * getPid() const { return m_pid; }

protected:
    unsigned m_eid;
    const TriangleGeometry<DataTypes> * m_geometry;
    unsigned m_pid[3];
    double m_fact[3];

};

}

}
