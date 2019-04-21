#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Internal iterator of elements
class SubsetElementIterator : public BaseElementIterator {
public:
    SubsetElementIterator(BaseElementContainer * geo, const std::set<unsigned> & subsetElements) : m_container(geo), m_subsetElements(subsetElements) {
        m_iterator = m_subsetElements.cbegin();
    }

    void next() {
        m_iterator++;
    }

    unsigned id() const {
        return *m_iterator;
    }

    bool end(unsigned ) const{
        return m_iterator==m_subsetElements.cend();
    }

    const BaseElement * element() const {
        return *(m_container->begin(id()));
    }

    BaseElementContainer * m_container;
    const std::set<unsigned> m_subsetElements;
    std::set<unsigned>::iterator m_iterator;
};



}

}
