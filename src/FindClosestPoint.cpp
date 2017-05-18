#ifndef SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H
#define SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H

#include "CollisionAlgorithm.h"
#include "ConstraintProximity.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "AABBDecorator.h"

namespace sofa {

namespace core {

namespace behavior {

#define min3(a,b,c) std::min(std::min(a,b),c)
#define max3(a,b,c) std::max(std::max(a,b),c)


ConstraintProximity findClosestElement(const defaulttype::Vector3 & T, BaseGeometry * geo) {
    ConstraintProximity min_pinfo;
    double minDist = 0;

    for(unsigned e=0;e<geo->getNbElements();e++) {
        ConstraintProximity pinfo;
        double dist = geo->projectPoint(e,T,pinfo);;

        if ((e==0) || (dist < minDist)) {
            min_pinfo = pinfo;
            minDist = dist;
        }
    }

    return min_pinfo;
}

ConstraintProximity findClosestProximity(const defaulttype::Vector3 & P,BaseGeometry * geo, const std::set<unsigned> & triangleSet) {
    ConstraintProximity min_pinfo;
    double minDist = 0;

    for(std::set<unsigned>::iterator it=triangleSet.begin();it!=triangleSet.end();++it) {
        unsigned tri = *it;

        ConstraintProximity pinfo;
        double dist = geo->projectPoint(tri,P,pinfo);

        if ((it==triangleSet.begin()) || (dist < minDist)) {
            min_pinfo = pinfo;
            minDist = dist;
        }
    }

    return min_pinfo;
}

void fillTriangleSet(AABBDecorator * decorator, int d,const defaulttype::Vec3i & cbox,std::set<unsigned> & triangleSet) {
    for (int i=-d;i<=d;i++) {
        if (cbox[0]+i < 0 || cbox[0]+i > decorator->d_nbox.getValue()[0]) continue;

        for (int j=-d;j<=d;j++) {
            if (cbox[1]+j < 0 || cbox[1]+j > decorator->d_nbox.getValue()[1]) continue;

            for (int k=-d;k<=d;k++) {
                if (cbox[2]+k < 0 || cbox[2]+k > decorator->d_nbox.getValue()[2]) continue;

                if (sqrt(i * i + j*j + k*k) > d) continue;

                const helper::vector<unsigned> & triangles = decorator->m_triangleboxes[cbox[0] + i][cbox[1] + j][cbox[2] + k];

                for (unsigned t=0;t<triangles.size();t++) triangleSet.insert(triangles[t]);
            }
        }
    }
}

ConstraintProximity findClosestElement(const defaulttype::Vector3 & T, BaseGeometry * geo, AABBDecorator * decorator) {
    defaulttype::Vec3i cbox;
    cbox[0] = floor((T[0] - decorator->m_Bmin[0])/decorator->m_cellSize[0]);
    cbox[1] = floor((T[1] - decorator->m_Bmin[1])/decorator->m_cellSize[1]);
    cbox[2] = floor((T[2] - decorator->m_Bmin[2])/decorator->m_cellSize[2]);

    //project P in the bounding box of the pbject
    //search with the closest box in bbox
    for (int i=0;i<3;i++) {
        if (cbox[i]<0) cbox[i] = 0;
        else if (cbox[i]>decorator->d_nbox.getValue()[i]) cbox[i] = decorator->d_nbox.getValue()[i];
    }

    unsigned max = max3(decorator->d_nbox.getValue()[0],
                        decorator->d_nbox.getValue()[1],
                        decorator->d_nbox.getValue()[2]);

    std::set<unsigned> triangleSet;

    unsigned d = 0;
    while (d<max && triangleSet.empty()) {
        fillTriangleSet(decorator,d,cbox,triangleSet);
        d++;
    }

    return findClosestProximity(T,geo,triangleSet);
}

ConstraintProximity CollisionAlgorithm::findClosestProximity(const defaulttype::Vector3 & T, BaseGeometry * geo) {
    AABBDecorator * aabb_decorator;
    geo->getContext()->get(aabb_decorator);

    ConstraintProximity res;
    if (aabb_decorator == NULL) res = findClosestElement(T,geo);
    else res = findClosestElement(T,geo,aabb_decorator);

    return res;
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H