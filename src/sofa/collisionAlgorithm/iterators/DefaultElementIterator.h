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
template<class CONTAINER>
class TDefaultElementIterator : public ElementIterator {
public:
    TDefaultElementIterator(const CONTAINER & c)
    : m_container(c)
    , m_it(0) {}

    void next() override { m_it++; }

    bool end() const override { return m_it>=m_container.size(); }

    virtual BaseElement::SPtr element() { return m_container[m_it]; }

    virtual const BaseElement::SPtr element() const { return m_container[m_it]; }

private:
    const CONTAINER & m_container;
    unsigned m_it;

};

}

}
