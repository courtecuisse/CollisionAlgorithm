#pragma once

#include <commonTypes.h>
#include <memory>
#include <map>
#include <vector>

namespace collisionAlgorithm {

class ConstraintElement;
class BaseGeometry;

class ConstraintProximity {
public :

    ConstraintProximity(ConstraintElement * elmt) : m_element(elmt) {}

    virtual Vector3 getPosition(TVecId v = TVecId::position) const = 0;

    virtual Vector3 getNormal() const = 0;

    virtual std::map<unsigned,Vector3> getContribution(const Vector3 & N) = 0;

    inline ConstraintElement * element() const {
        return (ConstraintElement*) m_element;
    }

protected:
    ConstraintElement * m_element;
};

typedef std::shared_ptr<ConstraintProximity> ConstraintProximityPtr;

class ConstraintElement {
    friend class ConstraintProximity;
public:

    ConstraintElement(BaseGeometry * geo) : m_geometry(geo) {}

    //this function returns a vector with all the control points of the element
    virtual ConstraintProximityPtr getControlPoint(const int i) = 0;

    // return the number of control points
    virtual unsigned getNbControlPoints() = 0;

    //this function project the point P on the element and return the corresponding proximity
    virtual ConstraintProximityPtr project(Vector3 P) = 0;

    virtual void draw(const std::vector<Vector3> & pos) = 0;

    template<class T = BaseGeometry>
    T * geometry() const {
        return (T*) m_geometry;
    }

protected:
    BaseGeometry * m_geometry;
};

typedef std::shared_ptr<ConstraintElement> ConstraintElementPtr;

class BaseGeometry : public BaseObject {
public:

    PortIn<Topology,REQUIRED> p_topology;
    PortIn<BaseObject> p_type;

    BaseGeometry()
    : p_topology(this)
    , p_type(this,"Any") {
        m_dirty = true;
    }

    void init() {
        m_elements.clear();
    }

    void beginStep() {
        if (m_dirty) {
            m_dirty = false;
            prepareDetection();
        }
    }

    void endStep() {
        m_dirty = true;
    }

    unsigned getNbElements() {
        beginStep();
        return m_elements.size();
    }

    static std::string getObjectType() {
        return std::string("Geometry");
    }

    ConstraintElementPtr getElement(unsigned i) const {
        return m_elements[i];
    }

    const std::vector<Vector3> & getPos(TVecId v) {
        return p_topology->p_state->get(v);
    }

    virtual void prepareDetection() {}

    void draw(const VisualParams * vparams);

protected:
    std::vector<ConstraintElementPtr> m_elements;
    bool m_dirty;

};




} // namespace controller
