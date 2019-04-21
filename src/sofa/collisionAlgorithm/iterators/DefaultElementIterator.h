#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
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
template<class CONTAINER>
class DefaultElementIterator : public BaseElementIterator {
public:

    /*!
     * \brief The BaseElement class is a basic abstract element container
     */
    class DefaultBaseElement : public BaseElement {
    public:

        DefaultBaseElement(const DefaultElementIterator * it) : m_iterator(it) {}

        virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
            return m_iterator->m_container->project(m_iterator,P);
        }

        virtual BaseProximity::SPtr center() const {
            return m_iterator->m_container->center(m_iterator);
        }

        virtual defaulttype::BoundingBox getBBox() const {
            return m_iterator->m_container->getBBox(m_iterator);
        }

        const DefaultElementIterator * m_iterator;
    };

    DefaultElementIterator(const CONTAINER * container, unsigned start)
    : m_container(container), m_id(start), m_accessor(this) {}

    virtual void next() {
        this->m_id++;
    }

    virtual bool end(unsigned sz) const {
        return m_id>=sz;
    }

    virtual unsigned id() const {
        return m_id;
    }

    virtual const BaseElement * element() const {
        return &m_accessor;
    }

    static BaseElementIterator::UPtr create(const CONTAINER * container, unsigned start = 0) {
        return BaseElementIterator::UPtr(new DefaultElementIterator<CONTAINER>(container, start));
    }

private:
    const CONTAINER * m_container;
    unsigned m_id;
    DefaultBaseElement m_accessor;
};

}

}
