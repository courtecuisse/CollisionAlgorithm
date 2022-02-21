#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class ProximityCreator {
public:
    virtual type::Vector3 getPosition(unsigned eid) = 0;
};

class BaseElement {
public:

    typedef std::shared_ptr<BaseElement> SPtr;

    virtual unsigned id() = 0;

    template<typename CAST>
    inline CAST * element_cast() {
        return (CAST*)this;
    }

    template<typename CAST>
    inline const CAST * element_cast() const {
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

//template<class TProxCreatorFunc>
//class TBaseElement : public BaseElement {
//public:
//    typedef TProxCreatorFunc ProxCreatorFunc;

//    TBaseElement(unsigned id, TProxCreatorFunc f) : m_eid(id), m_createProxFunc(f) {}

//    inline void setProximityCreator(ProxCreatorFunc f) { m_createProxFunc = f; }

//    unsigned id() override {
//        return m_eid;
//    }

//protected:
//    unsigned m_eid;
//    ProxCreatorFunc m_createProxFunc;
//};

}
