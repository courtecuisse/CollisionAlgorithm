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
template<class GEOMETRY, class PROXIMITYDATA, CONTROL_POINT CONTROL_SIZE>
class TDefaultElementIterator : public BaseElementIterator {
public:

    TDefaultElementIterator(const GEOMETRY * geometry, unsigned size, unsigned start)
    : m_geometry(geometry)
    , m_size(size)
    , m_it(start) {}

    void next() override {
        this->m_it++;
    }

    bool end() const override {
        return m_it >= m_size;
    }

    unsigned id() const override {
        return m_it;
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const override {
        return createSPtr(m_geometry,
                          m_geometry->project(P, m_it));
    }

    BaseProximity::SPtr createProximity(CONTROL_POINT pid = -1) const override {
        return createSPtr(m_geometry,
                          m_geometry->createProximity(m_it, pid)); // initialized with the center of the element
    }

    unsigned elementSize() const override {
        return CONTROL_SIZE;
    }

private:
    const GEOMETRY * m_geometry;
    unsigned m_it;
    const unsigned m_size;


    inline BaseProximity::SPtr createSPtr(const GEOMETRY * container, const PROXIMITYDATA & data) const {
        return BaseProximity::SPtr(new TBaseProximity<GEOMETRY, PROXIMITYDATA>(container,data));
    }
};

template<class PROXIMITYDATA>
class DefaultElementIterator {
public:

    template<class GEOMETRY, class ELEMENT>
    static BaseElementIterator::UPtr create(const GEOMETRY * c, const helper::vector<ELEMENT> & elmt, unsigned start = 0) {
        return BaseElementIterator::UPtr(new TDefaultElementIterator<GEOMETRY, PROXIMITYDATA, PROXIMITYDATA::nbControlPoints()>(c, elmt.size(), start));
    }

    template<class GEOMETRY>
    static BaseElementIterator::UPtr create(const GEOMETRY * c, unsigned size, unsigned start = 0) {
        return BaseElementIterator::UPtr(new TDefaultElementIterator<GEOMETRY, PROXIMITYDATA, PROXIMITYDATA::nbControlPoints()>(c, size, start));
    }
};

}

}
