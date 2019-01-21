#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseGeometry;

template<class ELMT_CONTAINER>
class DefaultElement : public BaseElement {
public:

    DefaultElement(unsigned id,const ELMT_CONTAINER * elmt) : m_id(id), m_elements(elmt) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        return m_elements->project(m_id, P);
    }

    inline BaseProximity::SPtr center() const {
        return m_elements->center(m_id);
    }

    inline defaulttype::BoundingBox getBBox() const {
        return m_elements->getBBox(m_id);
    }

protected:
    unsigned m_id;
    const ELMT_CONTAINER * m_elements;
};

template<class ELMT_CONTAINER>
class DefaultElementIterator : public BaseElementIterator {
    friend class Iterator;

public:
    DefaultElementIterator(const ELMT_CONTAINER * elmt, unsigned start, unsigned end) {
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
        return BaseElement::UPtr(new DefaultElement<ELMT_CONTAINER>(m_id,m_elements));
    }

    static BaseElementIterator::UPtr create(const ELMT_CONTAINER * elmt, unsigned end, unsigned start = 0) {
        return BaseElementIterator::UPtr(new DefaultElementIterator(elmt,start,end));
    }

private:
    unsigned m_id;
    unsigned m_end;
    const ELMT_CONTAINER * m_elements;
};

}

}
