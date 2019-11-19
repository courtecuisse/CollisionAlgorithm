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
template<class CONTAINER, class PROXIMITYDATA, class ELEMENT>
class TDefaultElementIterator : public BaseElementIterator {
public:

    TDefaultElementIterator(const CONTAINER * container, const helper::vector<ELEMENT> & elmts, unsigned start)
    : m_container(container)
    , m_elements(elmts)
    , m_id(start) {}

    void next() override {
        this->m_id++;
    }

    bool end() const override {
        return m_id >= m_elements.size();
    }

    unsigned id() const override {
        return m_id;
    }

    inline BaseProximity::SPtr createProximity(const CONTAINER * container, const PROXIMITYDATA & data) const {
        return BaseProximity::SPtr(new TBaseProximity<CONTAINER, PROXIMITYDATA>(container,data));
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const override {
        return createProximity(m_container,m_container->project(m_id, m_elements[m_id], P));
    }

    BaseProximity::SPtr center() const override {
        return createProximity(m_container,m_container->center(m_id, m_elements[m_id])); // initialized with the center of the element
    }

    defaulttype::BoundingBox getBBox() const override {
        return m_container->getBBox(m_elements[m_id]);
    }

private:
    const CONTAINER * m_container;
    const helper::vector<ELEMENT> & m_elements;
    unsigned m_id;
};

template<class PROXIMITYDATA>
class DefaultElementIterator {
public:
    template<class CONTAINER, class ELEMENT>
    static BaseElementIterator::UPtr create(const CONTAINER * c, const helper::vector<ELEMENT> & elmt, unsigned start = 0) {
        return BaseElementIterator::UPtr(new TDefaultElementIterator<CONTAINER, PROXIMITYDATA, ELEMENT>(c, elmt, start));
    }
};

}

}
