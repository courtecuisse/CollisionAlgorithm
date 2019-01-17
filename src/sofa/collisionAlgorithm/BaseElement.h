#pragma once

#include <memory>
#include <map>
#include <vector>
#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseElement {
public:
    class Iterator : public std::unique_ptr<BaseElement> {
    public:
        Iterator(BaseElement * ptr) : std::unique_ptr<BaseElement>(ptr) {}

        bool operator != (const BaseGeometry * geo) {
            return ! this->get()->end(geo);
        }

        void operator++() {
            this->get()->next();
        }

        void operator++(int) {
            this->get()->next();
        }
    };

    virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const = 0;

    virtual BaseProximity::SPtr center() const = 0;

    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual bool end(const BaseGeometry * geo) const = 0;

    virtual void next() = 0;

    virtual unsigned id() const = 0;
};

}

}
