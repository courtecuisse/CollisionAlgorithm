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

    virtual BaseElement * element() = 0;

    virtual const BaseElement * element() const = 0;

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

    BaseElement * element() override { return NULL; }

    const BaseElement * element() const override { return NULL; }

    size_t getOperationsHash() const override {
        return typeid(EmptyIterator).hash_code();
    }
};

template<class CONTAINER>
class TDefaultElementIteratorSPtr : public ElementIterator {
public:
    TDefaultElementIteratorSPtr(const CONTAINER & c,unsigned id=0) {
//        m_it = c.cbegin() + id;
        m_end = c.cend();

        auto it = c.cbegin();
        for (unsigned i=0; i<id; i++) {
            if (it == m_end) break;
            it++;
        }
        m_it = it;
    }

    void next() override { m_it++; }

    bool end() const override { return m_it==m_end; }

    size_t getOperationsHash() const override {
        if (m_it == m_end) return typeid(EmptyIterator).hash_code();
        else return (*m_it)->getOperationsHash();
    }

    virtual BaseElement* element() { return m_it->get(); }

    virtual const BaseElement* element() const { return m_it->get(); }

private:
    typename CONTAINER::const_iterator m_it;
    typename CONTAINER::const_iterator m_end;
};


template<class CONTAINER>
class TDefaultElementIteratorPtr : public ElementIterator {
public:
    TDefaultElementIteratorPtr(const CONTAINER & c,unsigned id=0) {
//        m_it = c.cbegin() + id;
        m_end = c.cend();

        auto it = c.cbegin();
        for (unsigned i=0; i<id; i++) {
            if (it == m_end) break;
            it++;
        }
        m_it = it;
    }

    void next() override { m_it++; }

    bool end() const override { return m_it==m_end; }

    size_t getOperationsHash() const override {
        if (m_it == m_end) return typeid(EmptyIterator).hash_code();
        else return (*m_it)->getOperationsHash();
    }

    virtual BaseElement * element() { return *m_it; }

    virtual const BaseElement * element() const { return *m_it; }

private:
    typename CONTAINER::const_iterator m_it;
    typename CONTAINER::const_iterator m_end;
};

ElementIterator::SPtr ElementIterator::empty(){
    return SPtr(new EmptyIterator());
}

}
