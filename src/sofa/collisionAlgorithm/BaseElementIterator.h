#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseGeometry;

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
        bool operator != (const BaseGeometry * ) {
            return ! this->get()->end();
        }

        //we take the geometry as parameter for std::iterator compatibility i.e. it != m_geo->end();
        bool operator== (const BaseGeometry * ) {
            return this->get()->end();
        }

        void operator++() {
            this->get()->next();
        }

        void operator++(int) {
            this->get()->next();
        }

        const BaseElementIterator * operator* () {
            return this->get();
        }

        const BaseElementIterator * operator* () const {
            return this->get();
        }
    };


    ///returns a new EmptyIterator
    static BaseElementIterator::UPtr empty() {
        class EmptyIterator : public BaseElementIterator {
        public:
            virtual bool end() const { return true; }

            virtual void next() {}

            virtual BaseProximity::SPtr project(const defaulttype::Vector3 &) const { return NULL; }

            virtual BaseProximity::SPtr center() const { return NULL; }

            virtual defaulttype::BoundingBox getBBox() const { return defaulttype::BoundingBox(); }

            virtual unsigned id() const { return 0; }
        };

        return UPtr(new EmptyIterator());
    }

    virtual bool end() const = 0;

    virtual void next() = 0;

    virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const = 0;

    virtual BaseProximity::SPtr center() const = 0;

    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual unsigned id() const = 0;

};

}

}
