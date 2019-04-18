#pragma once

#include <sofa/collisionAlgorithm/BaseDistanceMeasure.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/gl/BasicShapesGL.h>
#include <sofa/helper/gl/BasicShapes.h>

namespace sofa {

namespace collisionAlgorithm {

class ProjectiveDistanceMeasure : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS(ProjectiveDistanceMeasure, core::objectmodel::BaseObject) ;

    Data<BaseDistanceMeasure> d_distance ;
    Data<defaulttype::Mat3x4d> d_projectionMatrix;

    ProjectiveDistanceMeasure ()
        : d_distance(
              initData(
                  &d_distance,
                  BaseDistanceMeasure(std::bind(&ProjectiveDistanceMeasure::getProjectiveDistance, this, std::placeholders::_1, std::placeholders::_2)),
                  "distance",
                  "distance measure data"))
        , d_projectionMatrix(initData(&d_projectionMatrix, "projectionMatrix", "projection matrix"))
    {}

protected :
    double getProjectiveDistance (defaulttype::Vec3 P, defaulttype::Vec3 Q) {
        // replay use case
        // P in(here the image plane) Q out(surface)
        const defaulttype::Mat3x4d & Projmat = d_projectionMatrix.getValue();

        double
            x = Projmat[0][0] * Q[0] + Projmat[0][1] * Q[1] + Projmat[0][2] * Q[2] + Projmat[0][3],
            y = Projmat[1][0] * Q[0] + Projmat[1][1] * Q[1] + Projmat[1][2] * Q[2] + Projmat[1][3],
            z = Projmat[2][0] * Q[0] + Projmat[2][1] * Q[1] + Projmat[2][2] * Q[2] + Projmat[2][3];
        double
            xp = Projmat[0][0] * P[0] + Projmat[0][1] * P[1] + Projmat[0][2] * P[2] + Projmat[0][3],
            yp = Projmat[1][0] * P[0] + Projmat[1][1] * P[1] + Projmat[1][2] * P[2] + Projmat[1][3],
            zp = Projmat[2][0] * P[0] + Projmat[2][1] * P[1] + Projmat[2][2] * P[2] + Projmat[2][3];

        // output 2D : x, y => Q coordinates in the image plane
        if (z != 0) { // perspective projection
            y /= z ;
            x /= z ;
        } // else we simply do an orthographic projection
        if (zp != 0) { // perspective projection
            yp /= zp ;
            xp /= zp ;
        } // else we simply do an orthographic projection

        // chi2 distance measure is equivalent to euclidian distance measure
        // difference is we don't compute square root
//        std::cout << x << ":" << xp << "/" << y << ":" << yp << std::endl ;
//        std::cout << ((x-xp)*(x-xp)) + ((y-yp)*(y-yp)) << std::endl ;
        return ((x-xp)*(x-xp)) + ((y-yp)*(y-yp)) ;
    }

    static void drawPoint (defaulttype::Vector3 p) {
        helper::gl::drawSphere(p, 10) ;
    }

} ;


}

}
