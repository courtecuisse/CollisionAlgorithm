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
#ifndef SOFA_COMPONENT_TRIANGLENONLINEARGEOMETRY_H
#define SOFA_COMPONENT_TRIANGLENONLINEARGEOMETRY_H

#include "TriangleGeometry.h"
#include "ConstraintProximity.h"
#include "CollisionAlgorithm.h"

namespace sofa {

namespace core {

namespace behavior {

class TriangleNonLinearGeometry : public TriangleGeometry
{
public:
    SOFA_CLASS(TriangleNonLinearGeometry , TriangleGeometry );

    typedef  TriangleGeometry Inherit;
    typedef typename defaulttype::Vector2 Vector2;
    typedef typename defaulttype::Vector3 Vector3;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;

    Data <unsigned> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <unsigned> d_draw_tesselation;

    TriangleNonLinearGeometry()
        : Inherit()
        , d_nonlin_max_it(initData(&d_nonlin_max_it, (unsigned) 20,"nonlin_max_it","Max iteration in the Newton Raphson solver used for projection of points on non linear triangle"))
        , d_nonlin_tolerance(initData(&d_nonlin_tolerance, (double) 0.001,"nonlin_tol","Tolerance in the Newton Raphson solver used for projection of points on non linear triangle"))
        , d_nonlin_threshold(initData(&d_nonlin_threshold, (double) 0.00001,"nonlin_th","Threshold in the Newton Raphson solver used for projection of points on non linear triangle"))
        , d_draw_tesselation(initData(&d_draw_tesselation, (unsigned) 0.0,"tesselation","Draw tesselated triangles"))
    {}

    ConstraintProximityPtr projectPoint(const defaulttype::Vector3 & s,unsigned eid) const {
        return CollisionAlgorithm::newtonTriangularIterations(s, eid, Inherit::projectPoint(s, eid), d_nonlin_max_it.getValue(), d_nonlin_tolerance.getValue(), d_nonlin_threshold.getValue());
    }

    virtual ConstraintProximityPtr getNonLinearTriangleProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) const = 0;

    void tesselate(const core::visual::VisualParams * vparams, unsigned level,int tid, const defaulttype::Vector3 & bary_A,const defaulttype::Vector3 & bary_B, const defaulttype::Vector3 & bary_C) {
        if (level >= d_draw_tesselation.getValue()) {
            const topology::BaseMeshTopology::Triangle & tri = this->getTopology()->getTriangle(tid);

            defaulttype::Vector3 pA = this->getTriangleProximity(tid,tri[0],bary_A[0],tri[1],bary_A[1],tri[2],bary_A[2])->getPosition();
            defaulttype::Vector3 pB = this->getTriangleProximity(tid,tri[0],bary_B[0],tri[1],bary_B[1],tri[2],bary_B[2])->getPosition();
            defaulttype::Vector3 pC = this->getTriangleProximity(tid,tri[0],bary_C[0],tri[1],bary_C[1],tri[2],bary_C[2])->getPosition();

            this->drawTriangle(vparams,pA,pB,pC);

            return;
        }

        defaulttype::Vector3 bary_D = (bary_A + bary_B)/2.0;
        defaulttype::Vector3 bary_E = (bary_A + bary_C)/2.0;
        defaulttype::Vector3 bary_F = (bary_B + bary_C)/2.0;

        defaulttype::Vector3 bary_G = (bary_A + bary_B + bary_C)/3.0;

        tesselate(vparams,level+1,tid,bary_A,bary_D,bary_G);
        tesselate(vparams,level+1,tid,bary_D,bary_B,bary_G);

        tesselate(vparams,level+1,tid,bary_G,bary_B,bary_F);
        tesselate(vparams,level+1,tid,bary_G,bary_F,bary_C);

        tesselate(vparams,level+1,tid,bary_G,bary_C,bary_E);
        tesselate(vparams,level+1,tid,bary_A,bary_G,bary_E);
    }

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

        glDisable(GL_LIGHTING);

        if (vparams->displayFlags().getShowWireFrame()) glBegin(GL_LINES);
        else {
            glEnable(GL_CULL_FACE);
            glBegin(GL_TRIANGLES);
        }

        for(int t=0;t<this->getTopology()->getNbTriangles();t++) {
            tesselate(vparams,0,t,defaulttype::Vector3(1,0,0),defaulttype::Vector3(0,1,0),defaulttype::Vector3(0,0,1));
        }

        glEnd();
    }
};

} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
