#pragma once

#include <sofa/collisionAlgorithm/container/DataBezierTriangleContainer.h>
#include <sofa/collisionAlgorithm/geometry/PhongTriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/BezierTriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class BezierTriangleGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef BezierTriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data <unsigned> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <unsigned> d_draw_tesselation;

    DataBezierTriangleContainer<GEOMETRY> d_triangles;

    BezierTriangleGeometry()
    : d_nonlin_max_it(initData(&d_nonlin_max_it,(unsigned) 20,"nonlin_max_it", "number of iterations"))
    , d_nonlin_tolerance(initData(&d_nonlin_tolerance,(double) 0.001,"nonlin_tol", "Tolerance"))
    , d_nonlin_threshold(initData(&d_nonlin_threshold,(double) 0.00001,"nonlin_th", "Threshold"))
    , d_draw_tesselation(initData(&d_draw_tesselation,(unsigned) 0.0, "tesselation", "Number of tesselation"))
    , d_triangles(initData(&d_triangles, "triangles", "Vector of Triangles"))
    {}

    void tesselate(unsigned level,int tid, const defaulttype::Vector3 & bary_A,const defaulttype::Vector3 & bary_B, const defaulttype::Vector3 & bary_C) {
        if (level >= d_draw_tesselation.getValue()) {

            const Triangle& triangle = this->d_triangles.getValue()[tid];

            TriangleProximity<DataTypes> proxA(this->getState(), tid, triangle[0],triangle[1],triangle[2], bary_A[0],bary_A[1],bary_A[2], d_triangles.m_point_normals);
            TriangleProximity<DataTypes> proxB(this->getState(), tid, triangle[0],triangle[1],triangle[2], bary_B[0],bary_B[1],bary_B[2], d_triangles.m_point_normals);
            TriangleProximity<DataTypes> proxC(this->getState(), tid, triangle[0],triangle[1],triangle[2], bary_C[0],bary_C[1],bary_C[2], d_triangles.m_point_normals);

            // draw Triangle

            double delta = 0.2;
            defaulttype::Vector4 color = this->d_color.getValue();

            glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
            glVertex3dv(proxA.getPosition().data());
            glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
            glVertex3dv(proxB.getPosition().data());
            glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
            glVertex3dv(proxC.getPosition().data());

            return;
        }

        defaulttype::Vector3 bary_D = (bary_A + bary_B)/2.0;
        defaulttype::Vector3 bary_E = (bary_A + bary_C)/2.0;
        defaulttype::Vector3 bary_F = (bary_B + bary_C)/2.0;

        defaulttype::Vector3 bary_G = (bary_A + bary_B + bary_C)/3.0;

        tesselate(level+1,tid,bary_A,bary_D,bary_G);
        tesselate(level+1,tid,bary_D,bary_B,bary_G);

        tesselate(level+1,tid,bary_G,bary_B,bary_F);
        tesselate(level+1,tid,bary_G,bary_F,bary_C);

        tesselate(level+1,tid,bary_G,bary_C,bary_E);
        tesselate(level+1,tid,bary_A,bary_G,bary_E);
    }

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);

        glBegin(GL_TRIANGLES);
        for (unsigned i=0;i<this->d_triangles.getValue().size();i++) {
            tesselate(0, i , defaulttype::Vector3(1,0,0),defaulttype::Vector3(0,1,0),defaulttype::Vector3(0,0,1));
        }
        glEnd();
    }
};

}

}
