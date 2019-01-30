#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class EdgeProximity : public TBaseProximity<GEOMETRY> {
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

    EdgeProximity(const GEOMETRY * geo,unsigned eid, unsigned p1,unsigned p2,double f1,double f2)
    : TBaseProximity<GEOMETRY>(geo) {
        m_eid = eid;

        m_pid[0] = p1;
        m_pid[1] = p2;

        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    inline defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_geometry->getState()->read(v);

        return pos[m_pid[0]] * m_fact[0] +
               pos[m_pid[1]] * m_fact[1];
    }

    inline defaulttype::Vector3 getNormal() const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_geometry->getState()->read(core::VecCoordId::position());

        sofa::core::topology::BaseMeshTopology::Edge edge = this->m_geometry->getEdges()[m_eid];

        return (pos[edge[1]] - pos[edge[0]]).normalized();
    }

    void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_pid[0], N * m_fact[0]);
        it.addCol(m_pid[1], N * m_fact[1]);
    }

protected:
    unsigned m_eid;
    unsigned m_pid[2];
    double m_fact[2];

};

}

}
