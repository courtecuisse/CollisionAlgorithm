#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class PointElement : public BaseElement {
public:
    typedef GEOMETRY TGeometry;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    PointElement(unsigned id,const GEOMETRY * geo) : m_id(id), m_geo(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & /*P*/) const {
        return BaseProximity::create<PointProximity<GEOMETRY> >(m_geo,m_id);
    }

    inline BaseProximity::SPtr center() const {
        return BaseProximity::create<PointProximity<GEOMETRY> >(m_geo,m_id);
    }

    inline defaulttype::BoundingBox getBBox() const {
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[m_id]);
        return bbox;
    }

protected:
    unsigned m_id;
    const GEOMETRY * m_geo;
};

}

}
