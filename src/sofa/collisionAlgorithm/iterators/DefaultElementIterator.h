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
    typedef BaseGeometry::index_type index_type;

    TDefaultElementIterator(const GEOMETRY * geometry, index_type size, index_type start)
    : m_geometry(geometry)
    , m_size(size)
    , m_it(start) {}

    void next() override {
        this->m_it++;
    }

    bool end() const override {
        return m_it >= m_size;
    }

    index_type id() const override {
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

    index_type elementSize() const override {
        return CONTROL_SIZE;
    }

private:
    const GEOMETRY * m_geometry;
    index_type m_it;
    const index_type m_size;


    inline BaseProximity::SPtr createSPtr(const GEOMETRY * container, const PROXIMITYDATA & data) const {
        return BaseProximity::SPtr(new TBaseProximity<GEOMETRY, PROXIMITYDATA>(container,data));
    }
};

template<class PROXIMITYDATA>
class DefaultElementIterator {
public:
typedef BaseGeometry::index_type index_type;

    template<class GEOMETRY, class ELEMENT>
    static BaseElementIterator::UPtr create(const GEOMETRY * c, const helper::vector<ELEMENT> & elmt, index_type start = 0) {
        return BaseElementIterator::UPtr(new TDefaultElementIterator<GEOMETRY, PROXIMITYDATA, PROXIMITYDATA::nbControlPoints()>(c, elmt.size(), start));
    }

    template<class GEOMETRY>
    static BaseElementIterator::UPtr create(const GEOMETRY * c, index_type size, index_type start = 0) {
        return BaseElementIterator::UPtr(new TDefaultElementIterator<GEOMETRY, PROXIMITYDATA, PROXIMITYDATA::nbControlPoints()>(c, size, start));
    }
};

}

}
