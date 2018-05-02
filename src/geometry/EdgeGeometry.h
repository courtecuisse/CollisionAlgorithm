#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class EdgeGeometry : public BaseGeometry {
    friend class EdgeElement;
public:
    Port<Topology,REQUIRED> p_topology;

    EdgeGeometry()
    : p_topology("topology",LEFT,this) {}

    void prepareDetection();

    void init();

    void draw(const VisualParams *vparams);

    inline ReadAccessor<Vector3> read(VecID v) {
        return p_topology->p_state->read(v);
    }
};

class EdgeElement : public ConstraintElement {
    friend class EdgeProximity;
    friend class EdgeGeometry;

public:

    EdgeElement(EdgeGeometry *geo, unsigned pid);

    ConstraintProximityPtr getControlPoint(const int i);

    ConstraintProximityPtr project(Vector3 /*P*/);

    EdgeGeometry * geometry();

protected:
    unsigned m_pid[2];
    unsigned m_eid;
};

class EdgeProximity : public ConstraintProximity {
public :
    EdgeProximity(EdgeElement *geo,double f1,double f2);

    Vector3 getPosition(VecID v) const;

    Vector3 getNormal() const;

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

    inline EdgeElement * element() const {
        return (EdgeElement*) m_element;
    }

protected:
    double m_fact[2];
};

}

