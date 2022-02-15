#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class BaseGeometry;

class BaseElement {
public:

    typedef std::shared_ptr<BaseElement> SPtr;

    virtual unsigned id() = 0;

    template<typename CAST>
    inline CAST * cast() {
        return (CAST*)this;
    }

    template<typename CAST>
    inline const CAST * cast() const {
        return (CAST*)this;
    }

    virtual void getControlProximities(std::vector<BaseProximity::SPtr> & res) const = 0;

};


class ElementIterator {
    typedef BaseProximity::Index Index;
public:

    typedef std::shared_ptr<ElementIterator> SPtr;

//    ///defines a unique pointer iterator of baseElements
//    class SPtr : public std::shared_ptr<ElementIterator> {
//    public:
//        SPtr(ElementIterator * ptr) : std::shared_ptr<ElementIterator>(ptr) {}

//        //we take the geometry as parameter for std::iterator compatibility i.e. it != m_geo->end();
//        bool operator != (const BaseGeometry * ) { return ! this->get()->end(); }

//        //we take the geometry as parameter for std::iterator compatibility i.e. it != m_geo->end();
//        bool operator== (const BaseGeometry * ) { return this->get()->end(); }

//        void operator++() { this->get()->next(); }

//        void operator++(int) { this->get()->next(); }
//    };

    //This is necessary to have the correct behavior with the delete !
    virtual ~ElementIterator() = default;

    virtual bool end() const = 0;

    virtual void next() = 0;

    virtual std::shared_ptr<BaseElement> element() = 0;

    virtual const std::shared_ptr<BaseElement> element() const = 0;

    template<class CAST>
    inline CAST * element_cast() {
        return this->element()->cast<CAST>();
    }

    ///returns a new EmptyIterator
    static inline ElementIterator::SPtr empty();

};

static inline bool operator != (ElementIterator::SPtr it, const BaseGeometry * /*geo*/) {
    return it->end();
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

ElementIterator::SPtr ElementIterator::empty(){
    return SPtr(new EmptyIterator());
}

template<class TProxCreatorFunc>
class TBaseElement : public BaseElement {
public:
    typedef TProxCreatorFunc ProxCreatorFunc;

    TBaseElement(unsigned id, TProxCreatorFunc f) : m_eid(id), m_createProxFunc(f) {}

    inline void setProximityCreator(ProxCreatorFunc f) { m_createProxFunc = f; }

    unsigned id() override {
        return m_eid;
    }

protected:
    unsigned m_eid;
    ProxCreatorFunc m_createProxFunc;
};

}
