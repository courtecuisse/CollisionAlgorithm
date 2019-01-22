#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Internal iterator of elements
class SubsetElementIterator : public BaseElementIterator {
public:
    SubsetElementIterator(const BaseGeometry * geo, const std::set<unsigned> & subsetElements) : m_geometry(geo), m_subsetElements(subsetElements) {
        m_iterator = m_subsetElements.cbegin();
    }

    void next() {
        m_iterator++;
    }

    unsigned id() const {
        return *m_iterator;
    }

    bool end(const BaseGeometry * /*geo*/) const{
        return m_iterator==m_subsetElements.cend();
    }

    BaseElement::UPtr element() {
        return *(m_geometry->begin(id()));
    }

    const BaseGeometry * m_geometry;
    const std::set<unsigned> m_subsetElements;
    std::set<unsigned>::iterator m_iterator;
};



}

}
