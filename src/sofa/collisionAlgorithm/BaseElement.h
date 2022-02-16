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

    virtual void draw(const core::visual::VisualParams * vparams) = 0;
};

class ElementCast {
public:
    ElementCast(BaseElement::SPtr e) : m_element(e) {}

    template <typename CAST>
    inline operator CAST * () {
        return (CAST *) m_element.get();
    }

private:
    BaseElement::SPtr m_element;
};

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
