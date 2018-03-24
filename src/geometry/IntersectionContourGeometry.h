#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class IntersectionContourGeometry : public BaseGeometry {
public:
        Data<Vector3> d_planePos;
        Data<Vector3> d_planeNormal;

        IntersectionContourGeometry();

        void prepareDetection();

//        void draw(const VisualParams * vparams);
};


class IntersectionContourElement : public ConstraintElement {
    friend class IntersectionContourProximity;
    friend class IntersectionContourGeometry;
public:
    IntersectionContourElement(IntersectionContourGeometry *geo, unsigned pid1, unsigned pid2,double f1,double f2);

    ConstraintProximityPtr getControlPoint(const int i);

    ConstraintProximityPtr project(Vector3 /*P*/);

    void draw(const std::vector<Vector3> & X);

protected:
    unsigned m_pid[2];
    double m_fact[2];
};

class IntersectionContourProximity : public ConstraintProximity {
public :
    IntersectionContourProximity(IntersectionContourElement *elmt);

    Vector3 getPosition(TVecId v) const;

    Vector3 getNormal() const;

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

    inline IntersectionContourElement * element() const {
        return (IntersectionContourElement*) m_element;
    }
};

}
