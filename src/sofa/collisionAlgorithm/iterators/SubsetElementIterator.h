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
    SubsetElementIterator(BaseGeometry * geo, const std::set<unsigned> & subsetElements)
    : m_container(geo)
    , m_subsetElements(subsetElements) {
        m_iterator = m_subsetElements.cbegin();
    }

    void next() override {
        m_iterator++;
    }

    unsigned id() const override {
        return *m_iterator;
    }

    bool end() const override {
        return m_iterator==m_subsetElements.cend();
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const override {
        return m_container->begin(id())->project(P);
    }

    BaseProximity::SPtr createProximity(int pid = -1) const override {
        return m_container->begin(id())->createProximity(pid);
    }

    unsigned elementSize() const override {
        return m_container->begin(id())->elementSize();
    }

private:
    BaseGeometry * m_container;
    const std::set<unsigned> m_subsetElements;
    std::set<unsigned>::iterator m_iterator;
};



}

}
