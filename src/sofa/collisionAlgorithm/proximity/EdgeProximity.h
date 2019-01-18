#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class EdgeProximity : public TBaseProximity<GEOMETRY>
{
public :
    typedef EdgeProximity<GEOMETRY> PROXIMITY;

    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    EdgeProximity(const GEOMETRY * geometry, unsigned p1,unsigned p2,double f1,double f2)
    : TBaseProximity<GEOMETRY>(geometry){
        m_pid[0] = p1;
        m_pid[1] = p2;
        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId v) const {
        return this->m_geometry->getPosition(v, m_pid[0], m_pid[1], m_fact[0], m_fact[1]);
    }

    defaulttype::Vector3 getNormal() const {
        return this->m_geometry->getNormal(m_pid[0], m_pid[1], m_fact[0], m_fact[1]);
    }

    virtual void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_pid[0], N * m_fact[0]);
        it.addCol(m_pid[1], N * m_fact[1]);
    }

    static BaseProximity::SPtr project(const EdgeGeometry<DataTypes>* geometry, unsigned eid, const defaulttype::Vector3 & P) {
        core::topology::BaseMeshTopology::Edge edge;
        defaulttype::Vector2 factor;
        geometry->projectLinear(eid, P, edge, factor);
        return BaseProximity::create<PROXIMITY>(geometry,edge[0],edge[1],factor[0],factor[1]);
    }

    static BaseProximity::SPtr center(const EdgeGeometry<DataTypes>* geometry, unsigned eid) {
        const core::topology::BaseMeshTopology::Edge edge = geometry->d_edges.getValue()[eid];
        return BaseProximity::create<PROXIMITY>(geometry,edge[0],edge[1],0.5,0.5);
    }

    static defaulttype::BoundingBox getBBox(const EdgeGeometry<DataTypes>* geometry, unsigned eid) {
        const core::topology::BaseMeshTopology::Edge edge = geometry->d_edges.getValue()[eid];
        const helper::ReadAccessor<DataVecCoord >& x = *geometry->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

protected:
    double m_pid[2];
    double m_fact[2];
};

}

}
