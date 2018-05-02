#pragma once

#include <geometry/TriangleGeometry.h>

namespace collisionAlgorithm {

class ProjectiveContourGeometry : public TriangleGeometry {
public:
        typedef TriangleGeometry Inherit;

        Data<Mat3x4> d_projectionMatrix;
        Data<double> d_epsilon;

        void prepareDetection();

        int getNbElements();

        double projectPoint(unsigned tid,const Vector3 & s,ConstraintProximity & pinfo);

        void draw(const VisualParams* vparams);

        Vector3 getNormal(const ConstraintProximity & pinfo);

        BaseDecorator * getDecorator();

protected:
        std::vector<bool> m_filteredTriangle;
        std::vector<int> m_filteredEdge;

        Mat3x3 m_C,m_iC;
        Vector3 m_A;

        ProjectiveContourGeometry();

        void filterTriangleFunctionWithAABB(AABBDecorator * aabb,const unsigned p, const Vector3 & A);

        void filterTriangleFunction(const unsigned p, const Vector3 &A);

        Vector2 project3d(const Vector3 & p);

        void fillTriangleSet(AABBDecorator * decorator, int d,const Vec3i & cbox,std::set<unsigned> & triangleSet);

};

}
