#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
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
class DefaultElementIterator : public BaseElementIterator {
public:

    typedef typename CONTAINER::PROXIMITYDATA PROXIMITYDATA;

    DefaultElementIterator(const CONTAINER * container, unsigned start)
    : m_container(container), m_id(start) {}

    virtual void next() {
        this->m_id++;
    }

    virtual bool end(unsigned sz) const {
        return m_id>=sz;
    }

    virtual unsigned id() const {
        return m_id;
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const override {
        return createProximity(m_container->project(id(),P));
    }

    BaseProximity::SPtr center() const override {
        return createProximity(m_container->center(id()));
    }

    defaulttype::BoundingBox getBBox() const override {
        return m_container->getBBox(id());
    }

    static BaseElementIterator::UPtr create(CONTAINER * container, unsigned start = 0) {
        container->updateContainer();
        return BaseElementIterator::UPtr(new DefaultElementIterator<CONTAINER>(container, start));
    }

private:
    const CONTAINER * m_container;
    unsigned m_id;

    inline BaseProximity::SPtr createProximity(const PROXIMITYDATA & data) const {
        return BaseProximity::SPtr(new TBaseProximity<CONTAINER>(m_container, data));
    }
};

}

}
