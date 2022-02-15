#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Internal iterator of elements
class SubsetElementIterator : public ElementIterator {
public:
    typedef BaseProximity::Index Index;

    SubsetElementIterator(BaseGeometry * geo, const std::set<Index> & subsetElements)
    : m_geometry(geo)
    , m_subsetElements(subsetElements) {
        m_iterator = m_subsetElements.cbegin();
    }

    void next() override {
        m_iterator++;
    }

    bool end() const override {
        return m_iterator==m_subsetElements.cend();
    }

    virtual BaseElement::SPtr element() {
        return m_geometry->getElement(*m_iterator);
    }

    virtual const BaseElement::SPtr element() const {
        return m_geometry->getElement(*m_iterator);
    }


private:
    BaseGeometry * m_geometry;
    const std::set<Index> m_subsetElements;
    std::set<Index>::iterator m_iterator;
};



}

}
