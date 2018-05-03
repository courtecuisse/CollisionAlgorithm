#pragma once

#include <commonTypes.h>
#include <memory>
#include <map>
#include <vector>
#include <element/BaseElement.h>

namespace collisionAlgorithm {

class BaseGeometry : public BaseObject {
public:

//    Port<Topology,REQUIRED> p_topology;
    Port<BaseObject> p_type;

    BaseGeometry()
//    : p_topology("topology",LEFT,this)
    : p_type("any",RIGHT,this) {
        m_dirty = true;
    }

    void init() {
        m_elements.clear();
    }

    virtual void beginStep() {
        if (! m_dirty) return;
        m_dirty = false;
        prepareDetection();
    }

    virtual void endStep() {
        m_dirty = true;
    }

    virtual void handleEvent(Event * e) {
        if (dynamic_cast<AnimateBeginEvent *>(e)) beginStep();
        else if (dynamic_cast<AnimateEndEvent *>(e)) endStep();
    }

    unsigned getNbElements() {        
        return m_elements.size();
    }

    ConstraintElementPtr getElement(unsigned i) const {
        return m_elements[i];
    }

    ConstraintProximityPtr project(const Vector3 & P) {
        double min_dist = std::numeric_limits<double>::max();
        ConstraintProximityPtr min_prox = NULL;

        for (unsigned i=0;i<m_elements.size();i++) {
            ConstraintProximityPtr pdest = m_elements[i]->project(P);
            double dist = (P - pdest->getPosition()).norm();
            if (dist<min_dist) {
                min_dist = dist;
                min_prox = pdest;
            }
        }

        return min_prox;
    }

    virtual ReadAccessor<Vector3> read(VecID v) = 0;


protected:
    std::vector<ConstraintElementPtr> m_elements;
    bool m_dirty;

    virtual void prepareDetection() {}
};




} // namespace controller
