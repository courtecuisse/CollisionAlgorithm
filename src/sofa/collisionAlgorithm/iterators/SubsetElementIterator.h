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
    typedef BaseProximity::index_type index_type;

    SubsetElementIterator(BaseGeometry * geo, const std::set<index_type> & subsetElements)
    : m_container(geo)
    , m_subsetElements(subsetElements) {
        m_iterator = m_subsetElements.cbegin();
    }

    void next() override {
        m_iterator++;
    }

    index_type id() const override {
        return *m_iterator;
    }

    bool end() const override {
        return m_iterator==m_subsetElements.cend();
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const override {
        return m_container->begin(id())->project(P);
    }

    BaseProximity::SPtr createProximity(CONTROL_POINT pid = CONTROL_DEFAULT) const override {
        return m_container->begin(id())->createProximity(pid);
    }

    index_type elementSize() const override {
        return m_container->begin(id())->elementSize();
    }

private:
    BaseGeometry * m_container;
    const std::set<index_type> m_subsetElements;
    std::set<index_type>::iterator m_iterator;
};



}

}
