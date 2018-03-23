#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    void prepareDetection();

};

class PointElement : public ConstraintElement {
    friend class PointProximity;

public:

    PointElement(PointGeometry *geo, unsigned pid);

    ConstraintProximityPtr getControlPoint(const int i);

    unsigned getNbControlPoints();

    ConstraintProximityPtr project(Vector3 /*P*/);

    void draw(const std::vector<Vector3> & X);

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
