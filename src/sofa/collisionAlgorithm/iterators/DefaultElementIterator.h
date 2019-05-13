#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
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
template<class CONTAINER, class PROXIMITYDATA>
class DefaultElementIterator : public BaseElementIterator {
public:

    DefaultElementIterator(CONTAINER * container, unsigned size, unsigned start)
    : BaseElementIterator(container)
    , m_container(container)
    , m_size(size)
    , m_id(start) {}

    void next() override {
        this->m_id++;
    }

    bool end() const override {
        return m_id >= m_size;
    }

    unsigned id() const override {
        return m_id;
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const override {
        PROXIMITYDATA data = PROXIMITYDATA::center(m_container, id()); // initialized with the center of the element
        m_container->project(data, P); // compute the projection
        return BaseProximity::SPtr(new TBaseProximity<CONTAINER, PROXIMITYDATA>(m_container, data));
    }

    BaseProximity::SPtr center() const override {
        PROXIMITYDATA data = PROXIMITYDATA::center(m_container, id()); // initialized with the center of the element
        return BaseProximity::SPtr(new TBaseProximity<CONTAINER, PROXIMITYDATA>(m_container, data));
    }

    defaulttype::BoundingBox getBBox() const override {
        return PROXIMITYDATA::getBBox(m_container, id());
    }

    static BaseElementIterator::UPtr create(CONTAINER * container, unsigned size, unsigned start) {
        return BaseElementIterator::UPtr(new DefaultElementIterator<CONTAINER,PROXIMITYDATA>(container, size, start));
    }

private:
    const CONTAINER * m_container;
    unsigned m_size;
    unsigned m_id;
};

}

}
