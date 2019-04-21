#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

/*!
 * \brief The BaseElement class is a basic abstract element container
 */
class BaseElement {
public:

    virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const = 0;

    virtual BaseProximity::SPtr center() const = 0;

    virtual defaulttype::BoundingBox getBBox() const = 0;

};

/*!
 * \brief The BaseElementIterator class defines an abstract iterator class for BaseElements
 */
class BaseElementIterator {
public:

    ///defines a unique pointer iterator of baseElements
    class UPtr : public std::unique_ptr<BaseElementIterator> {
    public:
        UPtr(BaseElementIterator * ptr) : std::unique_ptr<BaseElementIterator>(ptr) {}

        //we take the geometry as parameter for std::iterator compatibility i.e. it != m_geo->end();
        bool operator != (const unsigned sz) {
            return ! this->get()->end(sz);
        }

//        bool operator == (const BaseGeometry * /*geo*/) {
//            return this->get()->end();
//        }

        void operator++() {
            this->get()->next();
        }

        void operator++(int) {
            this->get()->next();
        }

        const BaseElement * operator* () {
            return this->get()->element();
        }

        const BaseElement * operator* () const {
            return this->get()->element();
        }
    };


    ///returns a new EmptyIterator
    static BaseElementIterator::UPtr empty() {
        class EmptyIterator : public BaseElementIterator {
        public:
            virtual bool end(unsigned ) const { return true; }

            virtual void next() {}

            virtual const BaseElement * element() const { return NULL; }

            virtual unsigned id() const { return 0; }
        };

        return UPtr(new EmptyIterator());
    }

    virtual bool end(unsigned sz) const = 0;

    virtual void next() = 0;

    virtual const BaseElement * element() const = 0;

    virtual unsigned id() const = 0;
};

}

}
