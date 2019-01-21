#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class DataEdgeElement : public DataElemnt<sofa::core::topology::BaseMeshTopology::Edge> {
public:

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

    explicit DataEdgeElement(const typename DataElemnt<sofa::core::topology::BaseMeshTopology::Edge>::InitData& init)
    : DataElemnt<sofa::core::topology::BaseMeshTopology::Edge>(init) {
        m_geometry = dynamic_cast<const GEOMETRY*>(init.owner);
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<DataEdgeElement<GEOMETRY> >::create(this, this->getValue().size(), eid);
    }

    virtual const BaseGeometry * end() const {
        return m_geometry;
    }

    inline BaseProximity::SPtr project(unsigned eid, const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Edge edge;
        defaulttype::Vector2 factor;
        project(eid, P, edge, factor);

        return BaseProximity::create<EdgeProximity<DataTypes> >(m_geometry->getState(),edge[0],edge[1],factor[0],factor[1]);
    }

    inline BaseProximity::SPtr center(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = this->getValue()[eid];
        return BaseProximity::create<EdgeProximity<DataTypes> >(m_geometry->getState(),edge[0],edge[1],0.5,0.5);
    }

    inline defaulttype::BoundingBox getBBox(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = this->getValue()[eid];
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geometry->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

protected:

    void project(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Edge & edge, defaulttype::Vector2 & factor) const {
        edge = this->getValue()[eid];

        const helper::ReadAccessor<Data <VecCoord> >& x = *m_geometry->getState()->read(core::VecCoordId::position());

        double fact_u;
        double fact_v;

        Coord v = x[edge[1]] - x[edge[0]];
        fact_v = dot (P - x[edge[0]],v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        factor[0] = fact_u;
        factor[1] = fact_v;
    }

    const GEOMETRY * m_geometry;
};


}

}
