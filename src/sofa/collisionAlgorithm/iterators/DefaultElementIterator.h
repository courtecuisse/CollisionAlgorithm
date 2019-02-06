#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class ELMT>
class DefaultElementIterator : public BaseElementIterator {
    friend class Iterator;

public:
    typedef typename ELMT::TGeometry GEOMETRY;

    DefaultElementIterator(const GEOMETRY * geo, unsigned start, unsigned end) {
        m_id = start;
        m_end = end;
        m_geo = geo;
    }

    virtual void next() {
        this->m_id++;
    }

    virtual bool end() const {
        return m_id>=m_end;
    }

    virtual unsigned id() const {
        return m_id;
    }

    BaseElement::UPtr element() {
        return BaseElement::UPtr(new ELMT(m_id,m_geo));
    }

    static BaseElementIterator::UPtr create(const GEOMETRY * geo, unsigned end, unsigned start = 0) {
        return BaseElementIterator::UPtr(new DefaultElementIterator(geo,start,end));
    }

private:
    unsigned m_id;
    unsigned m_end;
    const GEOMETRY * m_geo;
};

}

}
