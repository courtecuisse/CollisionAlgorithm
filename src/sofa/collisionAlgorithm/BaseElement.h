#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

//class ProximityCreator {
//public:
//    virtual type::Vector3 getPosition(unsigned eid) = 0;
//};

class ControlPoints {
public:

    void insert(const ControlPoints & cp) {
        if ((m_points.size()==0) && (cp.m_points.size()>0)) m_points.push_back(cp.m_points[0]);

        for (unsigned i=0;i<cp.m_points.size();i++) {
            unsigned count = 0;
            for (unsigned j=0;j<m_points.size();j++) {
                if (m_points[j] == cp.m_points[i]) /*return*/ count++;
            }
            if (count > 0) continue;

            m_points.push_back(cp.m_points[i]);
        }
    }

    void insert(BaseProximity::SPtr prox) {
        for (unsigned i=0; i<m_points.size(); i++) {
            if (m_points[i] == prox) return;

            m_points.push_back(prox);
        }
        if (m_points.size()==0) m_points.push_back(prox);
    }

    void insert(std::vector<BaseProximity::SPtr> points) {
        if ((m_points.size()==0) && (points.size()>0)) m_points.push_back(points[0]);

        for (unsigned i=0; i<points.size(); i++) {
            unsigned count = 0;
            for (unsigned j=0; j<m_points.size(); j++) {
                if (m_points[j] == points[i]) /*return*/ count++;
            }
            if (count > 0) continue;
            m_points.push_back(points[i]);
        }
    }

    BaseProximity::SPtr operator[] (int i) const {
        return m_points[i];
    }

    std::vector<BaseProximity::SPtr> & getProximities() {
        return m_points;
    }

private:
    std::vector<BaseProximity::SPtr> m_points;
};



class BaseElement {
public:

    typedef std::shared_ptr<BaseElement> SPtr;

//    virtual unsigned id() = 0;

    template<typename CAST>
    inline CAST * element_cast() {
        return (CAST*)this;
    }

    template<typename CAST>
    inline const CAST * element_cast() const {
        return (CAST*)this;
    }

    /*virtual*/ ControlPoints & getControlProximities() {
        return m_controlPoints;
    }

    virtual void getSubElements(std::set<BaseElement::SPtr> & subElem) const = 0;

    virtual void draw(const core::visual::VisualParams * vparams) = 0;

    virtual size_t getOperationsHash() const = 0;

    virtual std::string name() const = 0;

    template<class ELEMENT,class... ARGS>
    static inline typename ELEMENT::SPtr create(ARGS... args) {
        return typename ELEMENT::SPtr(new ELEMENT(args...));
    }

private:
    ControlPoints m_controlPoints;

};

class ElementCast {
public:
    ElementCast(BaseElement* e) : m_element(e) {}

    template <typename CAST>
    inline operator CAST * () {
        return (CAST *) m_element;
    }

private:
    BaseElement* m_element;
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
