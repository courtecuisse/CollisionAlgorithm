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
    typedef BaseProximity::Index Index;

    SubsetElementIterator(BaseGeometry * geo, const std::set<Index> & subsetElements)
    : m_container(geo)
    , m_subsetElements(subsetElements) {
        m_iterator = m_subsetElements.cbegin();
    }

    void next() override {
        m_iterator++;
    }

    Index id() const override {
        return *m_iterator;
    }

    bool end() const override {
        return m_iterator==m_subsetElements.cend();
    }

    BaseProximity::SPtr createProximity(CONTROL_POINT pid = CONTROL_DEFAULT) const override {
        return m_container->begin(id())->createProximity(pid);
    }

    Index elementSize() const override {
        return m_container->begin(id())->elementSize();
    }

private:
    BaseGeometry * m_container;
    const std::set<Index> m_subsetElements;
    std::set<Index>::iterator m_iterator;
};



}

}
