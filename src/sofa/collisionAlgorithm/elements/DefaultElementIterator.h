#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class DefaultElement : public BaseElementIterator {
    friend class Iterator;

public:
    DefaultElement(unsigned id, unsigned end,const BaseGeometry * geo) {
        m_id = id;
        m_end = end;
        m_geometry = geo;
    }


    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        return m_geometry->project(m_id, P);
    }

    inline BaseProximity::SPtr center() const {
        return m_geometry->center(m_id);
    }

    inline defaulttype::BoundingBox getBBox() const {
        return m_geometry->getBBox(m_id);
    }

    virtual void next() {
        this->m_id++;
    }

    virtual bool end(const BaseGeometry * /*geo*/) const {
        return m_id>=m_end;
    }

    virtual unsigned id() const {
        return m_id;
    }

private:
    unsigned m_id;
    unsigned m_end;
    const BaseGeometry * m_geometry;
};



}

}
