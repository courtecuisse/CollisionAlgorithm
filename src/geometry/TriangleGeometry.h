#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class TriangleGeometry : public BaseGeometry {
    friend class TriangleElement;
    friend class TriangleProximity;

public:

    Port<Topology,REQUIRED> p_topology;

    TriangleGeometry()
    : p_topology("topology",LEFT,this) {}

    void init();

    void prepareDetection();

    void draw(const VisualParams *vparams);

    typedef struct {
        Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        Vector3 tn,ax1,ax2;
    } TriangleInfo;

    inline ReadAccessor<Vector3> read(VecID v) {
        return p_topology->p_state->read(v);
    }

protected:
    std::vector<TriangleInfo> m_triangle_info;
    std::vector<Vector3> m_pointNormal;
};

class TriangleElement : public ConstraintElement {
    friend class TriangleProximity;
    friend class TriangleGeometry;

public:

    TriangleElement(TriangleGeometry *geo, unsigned pid);

    ConstraintProximityPtr getControlPoint(const int i);

    void computeBaryCoords(const Vector3 & proj_P,const TriangleGeometry::TriangleInfo & tinfo, const Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const;

    ConstraintProximityPtr project(Vector3 /*P*/);

    inline TriangleGeometry * geometry() const {
        return (TriangleGeometry*) m_geometry;
    }

protected:
    Vector3 m_pos;
    unsigned m_pid[3];
    unsigned m_eid;
};

class TriangleProximity : public ConstraintProximity {
public :
    TriangleProximity(TriangleElement *geo,double f1,double f2,double f3);

    Vector3 getPosition(VecID v) const;

    Vector3 getNormal() const;

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

    inline TriangleElement * element() const {
        return (TriangleElement*) m_element;
    }

protected:
    double m_fact[3];
};

}

