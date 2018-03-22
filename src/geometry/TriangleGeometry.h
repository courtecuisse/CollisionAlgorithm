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

namespace graFE {

class TriangleGeometry : public BaseGeometry {
    friend class TriangleElement;
    friend class TriangleProximity;

public:

    void createElements();

    void beginStep();

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

    unsigned getNbControlPoints();

    void computeBaryCoords(const Vector3 & proj_P,const TriangleGeometry::TriangleInfo & tinfo, const Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const;

    ConstraintProximityPtr project(Vector3 /*P*/);

    void draw(const std::vector<Vector3> & X);

protected:
    unsigned m_pid[3];
    unsigned m_eid;
    TriangleGeometry * m_geo;
};

class TriangleProximity : public ConstraintProximity {
public :
    TriangleProximity(TriangleElement *geo,double f1,double f2,double f3);

    Vector3 getPosition() const;

    Vector3 getFreePosition() const;

    Vector3 getNormal() const;

    ConstraintElement * getElement();



protected:
    TriangleElement * m_elmt;
    double m_fact[3];

};


//class TriangleGeometry : public EdgeGeometry
//{
//    friend class TriangleElement;
//    friend class TriangleProximity;

//public:
//    SOFA_CLASS(TriangleGeometry , EdgeGeometry );

//    Data<bool> d_phong;

//    TriangleGeometry();

//    ConstraintProximityPtr getTriangleProximity(unsigned eid, unsigned p1, double f1, unsigned p2, double f2, unsigned p3, double f3) const;

////    void projectPoint(const defaulttype::Vector3 & s,TriangleConstraintProximity * pinfo) const;

//    void draw(const core::visual::VisualParams */*vparams*/);

//    int getNbTriangles() const;

//    unsigned getNbElements() const;

//    ConstraintElementPtr getElement(unsigned eid) const;

//protected:



//    virtual void prepareDetection();

//    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_w,double & fact_u, double & fact_v) const;



//    void drawTriangle(const core::visual::VisualParams * vparams,const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C);
//};

}

