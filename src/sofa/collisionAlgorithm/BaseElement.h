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
    friend class Iterator;

public:
    class Iterator : public std::unique_ptr<BaseElement> {
    public:
        Iterator(BaseElement* ptr) : std::unique_ptr<BaseElement>(ptr) {}

        // for iterator compatibility i.e. it != geo.end()
        bool operator != (const BaseGeometry * /*geo*/) {
            return ! end();
        }

        void operator++(int) {
            this->get()->next();
        }

        bool end() {
            return this->get()->end();
        }
    };

    virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const = 0;

    virtual BaseProximity::SPtr center() const = 0;

    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual unsigned id() const = 0;

protected:
    virtual bool end() const = 0;

    virtual void next() = 0;
};


class DefaultElement : public BaseElement {
public:
    class Iterator : public BaseElement::Iterator {
    public:
        Iterator(unsigned id, DefaultElement * ptr) : BaseElement::Iterator(ptr) {
            ptr->m_id = id;
        }
    };

    virtual void next() {
        this->m_id++;
    }

    virtual bool end() const = 0;

    virtual unsigned id() const {
        return m_id;
    }

private:
    unsigned m_id;
};




}

}
