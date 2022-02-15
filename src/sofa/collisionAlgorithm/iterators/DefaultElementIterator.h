#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <memory>
#include <functional>
#include <iostream>
#include <algorithm>

namespace sofa
{

namespace collisionAlgorithm
{

/*!
 * \brief The BaseElement class is a basic abstract element container
 */
class DefaultElementIterator : public ElementIterator {
public:
    typedef BaseGeometry::Index Index;

    DefaultElementIterator(const BaseGeometry * geo)
    : m_geometry(geo)
    , m_it(0) {}

    void next() override { m_it++; }

    bool end() const override { return m_it>=m_geometry->elementSize(); }

    virtual BaseElement::SPtr element() {
        return m_geometry->getElement(m_it);
    }

    virtual const BaseElement::SPtr element() const { return m_geometry->getElement(m_it); }

private:
    const BaseGeometry * m_geometry;
    unsigned m_it;

};

}

}
