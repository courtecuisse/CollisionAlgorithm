#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class EdgeGeometry : public BaseGeometry {
    friend class EdgeElement;
public:
    void prepareDetection();

    void init();

    void draw(const VisualParams *vparams);
};

class EdgeElement : public ConstraintElement {
    friend class EdgeProximity;
    friend class EdgeGeometry;

public:

    EdgeElement(EdgeGeometry *geo, unsigned pid);

    ConstraintProximityPtr getControlPoint(const int i);

    ConstraintProximityPtr project(Vector3 /*P*/);

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

