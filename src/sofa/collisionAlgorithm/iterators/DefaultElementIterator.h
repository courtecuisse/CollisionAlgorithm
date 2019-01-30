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

    DefaultElementIterator(const GEOMETRY * elmt, unsigned start, unsigned end) {
        m_id = start;
        m_end = end;
        m_elements = elmt;
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

    BaseElement::UPtr element() {
        return BaseElement::UPtr(new ELMT(m_id,m_elements));
    }

    static BaseElementIterator::UPtr create(const GEOMETRY * elmt, unsigned end, unsigned start = 0) {
        return BaseElementIterator::UPtr(new DefaultElementIterator(elmt,start,end));
    }

private:
    unsigned m_id;
    unsigned m_end;
    const GEOMETRY * m_elements;
};

}

}
