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

    inline ElementCast element_cast() {
        return ElementCast(element());
    }

    ///returns a new EmptyIterator
    static inline ElementIterator::SPtr empty();

    template<class T>
    static inline ElementIterator::SPtr defaultIterator(const T & c);

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

    virtual BaseElement::SPtr element() { return *m_it; }

    virtual const BaseElement::SPtr element() const { return *m_it; }

private:
    typename CONTAINER::const_iterator m_it;
    typename CONTAINER::const_iterator m_end;
};

ElementIterator::SPtr ElementIterator::empty(){
    return SPtr(new EmptyIterator());
}


template<class T>
ElementIterator::SPtr defaultIterator(const T & c){
    return SPtr(new TDefaultElementIterator<T>(c));
}

}
