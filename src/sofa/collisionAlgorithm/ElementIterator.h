#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class BaseGeometry;

class ElementIterator {
public:

    typedef std::shared_ptr<ElementIterator> SPtr;

    //This is necessary to have the correct behavior with the delete !
    virtual ~ElementIterator() = default;

    virtual bool end() const = 0;

    virtual void next() = 0;

    virtual std::shared_ptr<BaseElement> element() = 0;

    virtual const std::shared_ptr<BaseElement> element() const = 0;

    virtual size_t getOperationsHash() const = 0;

    inline ElementCast element_cast() {
        return ElementCast(element());
    }

    ///returns a new EmptyIterator
    static inline ElementIterator::SPtr empty();

};

static inline bool operator != (ElementIterator::SPtr it, const BaseGeometry * /*geo*/) {
    return ! it->end();
}

static inline void operator ++ (ElementIterator::SPtr it) {
    return it->next();
}

static inline void operator ++ (ElementIterator::SPtr it, int /*NB*/) {
//    for (int i=0;i<NB;i++)
        it->next();
}

class EmptyIterator : public ElementIterator {
public:
    bool end() const override { return true; }

    void next() override {}

    std::shared_ptr<BaseElement> element() override { return NULL; }

    const std::shared_ptr<BaseElement> element() const override { return NULL; }

    size_t getOperationsHash() const override {
        return typeid(EmptyIterator).hash_code();
    }
};

template<class CONTAINER>
class TDefaultElementIterator : public ElementIterator {
public:
    TDefaultElementIterator(const CONTAINER & c) {
        m_it = c.cbegin();
        m_end = c.cend();
    }

    void next() override { m_it++; }

    bool end() const override { return m_it==m_end; }

    size_t getOperationsHash() const override {
        if (m_it == m_end) return typeid(EmptyIterator).hash_code();
        else return (*m_it)->getOperationsHash();
    }

    virtual BaseElement::SPtr element() { return *m_it; }

    virtual const BaseElement::SPtr element() const { return *m_it; }

private:
    typename CONTAINER::const_iterator m_it;
    typename CONTAINER::const_iterator m_end;
};

ElementIterator::SPtr ElementIterator::empty(){
    return SPtr(new EmptyIterator());
}

}
