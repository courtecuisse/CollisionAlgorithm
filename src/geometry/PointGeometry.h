#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    void createElements();

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
    PointGeometry * m_geo;
};

class PointProximity : public ConstraintProximity {
public :
    PointProximity(PointElement *geo);

    Vector3 getPosition() const;

    Vector3 getFreePosition() const;

    Vector3 getNormal() const;

    ConstraintElement * getElement();

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

protected:
    PointElement * m_elmt;
};


}
