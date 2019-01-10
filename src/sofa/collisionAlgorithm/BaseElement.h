#pragma once

#include <memory>
#include <map>
#include <vector>
#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class ElementIterator {
public:

    typedef unsigned End;

    class UPtr : public std::unique_ptr<ElementIterator> {
    public:

        UPtr(ElementIterator * ptr) : std::unique_ptr<ElementIterator>(ptr) {}

        bool operator != (End end) {
            return this->get()->m_id < end;
        }

        UPtr& operator++() {
            this->get()->m_id++;
            return *this;
        }

        void operator++(int /*n*/) {
            this->get()->m_id++;
        }
    };

    ElementIterator() {
        m_id = 0;
    }

    virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const = 0;

    virtual BaseProximity::SPtr center() const = 0;

    unsigned id() const {
        return m_id;
    }


private:
    unsigned m_id;
};


}

}
