#pragma once

#include <memory>
#include <map>
#include <vector>
#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

//this class declar all the virtual classes that needs to be redefined for Elements
class VirtalElement {
public:
    virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const = 0;

    virtual BaseProximity::SPtr center() const = 0;

    virtual defaulttype::BoundingBox getBBox() const = 0;

};

class BaseElement : public VirtalElement {
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

    virtual bool end(const BaseGeometry * geo) const = 0;

    virtual void next() = 0;

    virtual unsigned id() const = 0;
};


class DefaultElement : public BaseElement {
public:

    class Iterator : public BaseElement::Iterator {
    public:
        Iterator(unsigned id, unsigned end, DefaultElement * ptr) : BaseElement::Iterator(ptr) {
            ptr->m_id = id;
            ptr->m_end = end;
        }
    };

    DefaultElement() {
        m_id = 0;
        m_end = 0;
    }

    virtual void next() {
        this->m_id++;
    }

    virtual bool end(const BaseGeometry * /*geo*/) const {
        return m_id>=m_end;
    }

    virtual unsigned id() const {
        return m_id;
    }

private:
    unsigned m_id;
    unsigned m_end;
};




}

}
