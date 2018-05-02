#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    Port<Topology,REQUIRED> p_topology;

    PointGeometry()
    : p_topology("topology",LEFT,this) {}


    void prepareDetection();

    void init();

    void draw(const VisualParams *vparams);

    inline ReadAccessor<Vector3> read(VecID v) {
        return p_topology->p_state->read(v);
    }

};

class PointElement : public ConstraintElement {
    friend class PointProximity;
    friend class PointGeometry;

public:

    PointElement(PointGeometry *geo, unsigned pid);

    ConstraintProximityPtr getControlPoint(const int i);

    ConstraintProximityPtr project(Vector3 /*P*/);

    PointGeometry * geometry();

protected:
    unsigned m_pid;
};

class PointProximity : public ConstraintProximity {
public :
    PointProximity(PointElement *geo);

    Vector3 getPosition(VecID v) const;

    Vector3 getNormal() const;

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

    inline PointElement * element() const {
        return (PointElement*) m_element;
    }
};


}
