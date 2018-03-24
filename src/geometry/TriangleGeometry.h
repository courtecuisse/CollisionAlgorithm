/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class TriangleGeometry : public BaseGeometry {
    friend class TriangleElement;
    friend class TriangleProximity;

public:

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
    unsigned m_pid[3];
    unsigned m_eid;
};

class TriangleProximity : public ConstraintProximity {
public :
    TriangleProximity(TriangleElement *geo,double f1,double f2,double f3);

    Vector3 getPosition(TVecId v) const;

    Vector3 getNormal() const;

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

    inline TriangleElement * element() const {
        return (TriangleElement*) m_element;
    }

protected:
    double m_fact[3];
};

}

