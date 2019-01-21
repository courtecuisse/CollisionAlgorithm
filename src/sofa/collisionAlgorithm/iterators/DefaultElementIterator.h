#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class ELMT>
class DefaultElementIterator : public BaseElementIterator {
    friend class Iterator;

public:
    DefaultElementIterator(const ELMT * geo, unsigned start, unsigned end) {
        m_id = start;
        m_end = end;
        m_elements = geo;
    }

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        return m_elements->project(m_id, P);
    }

    inline BaseProximity::SPtr center() const {
        return m_elements->center(m_id);
    }

    inline defaulttype::BoundingBox getBBox() const {
        return m_elements->getBBox(m_id);
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

    static BaseElementIterator::UPtr create(const ELMT * elmt, unsigned end, unsigned start = 0) {
        return BaseElementIterator::UPtr(new DefaultElementIterator(elmt,start,end));
    }

private:
    unsigned m_id;
    unsigned m_end;
    const ELMT * m_elements;
};

}

}
