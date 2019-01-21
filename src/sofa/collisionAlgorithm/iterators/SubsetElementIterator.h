#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/BaseElementContainer.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Internal iterator of elements
class SubsetElementIterator : public BaseElementIterator {
public:
    SubsetElementIterator(const BaseDataElmtContainer * elmt, const std::set<unsigned> & subsetElements) : m_elements(elmt), m_subsetElements(subsetElements) {
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
        return m_elements->begin(id())->element();
    }

    const BaseDataElmtContainer * m_elements;
    const std::set<unsigned> m_subsetElements;
    std::set<unsigned>::iterator m_iterator;
};



}

}
