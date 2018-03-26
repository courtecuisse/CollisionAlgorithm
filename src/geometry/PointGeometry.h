#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:

    void prepareDetection();

    void init();

    void draw(const VisualParams *vparams);

};

class PointElement : public ConstraintElement {
    friend class PointProximity;
    friend class PointGeometry;

public:

    PointElement(PointGeometry *geo, unsigned pid);

    ConstraintProximityPtr getControlPoint(const int i);

    ConstraintProximityPtr project(Vector3 /*P*/);

protected:
    unsigned m_pid;
};

class PointProximity : public ConstraintProximity {
public :
    PointProximity(PointElement *geo);

    Vector3 getPosition(TVecId v) const;

    Vector3 getNormal() const;

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

    inline PointElement * element() const {
        return (PointElement*) m_element;
    }
};


}
