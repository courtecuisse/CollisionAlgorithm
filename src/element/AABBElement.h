#pragma once

#include <BaseGeometry.h>
#include <geometry/AABBGeometry.h>

namespace collisionAlgorithm {

class AABBElement : public ConstraintElement {
public:

    class AABBProximity : public ConstraintProximity {
    public :
        AABBProximity(AABBElement * elmt,Vector3 P) : ConstraintProximity(elmt) {
            m_position = P;
        }

        Vector3 getPosition(VecCoordId /*v*/) const {
            return m_position;
        }

        Vector3 getNormal() const {
            return Vector3(0,0,0);
        }

        std::map<unsigned,double> getContributions() {
            std::map<unsigned,double> res;
            return res;
        }

        inline AABBElement * element() const {
            return (AABBElement*) m_element;
        }

        Vector3 m_position;
    };

    AABBElement(AABBGeometry * geo, unsigned _i, unsigned _j, unsigned _k)
    : ConstraintElement(geo,1)
    , i(_i)
    , j(_j)
    , k(_k) {

        Vector3 P = (geometry()->m_Bmin + Vector3((i  ) * geometry()->m_cellSize[0],(j  ) * geometry()->m_cellSize[1],(k  ) * geometry()->m_cellSize[2]) +
                     geometry()->m_Bmin + Vector3((i+1) * geometry()->m_cellSize[0],(j+1) * geometry()->m_cellSize[1],(k+1) * geometry()->m_cellSize[2])) *
                    0.5;

        m_controlPoint = std::make_shared<AABBProximity>(this,P);
//        m_controlPoint = project(P);
    }

    //this function returns a vector with all the control points of the element
    ConstraintProximityPtr getControlPoint(const int /*cid*/) {
        return m_controlPoint;
    }

    //this function project the point P on the element and return the corresponding proximity
    ConstraintProximityPtr project(Vector3 /*P*/) {
//        std::cout << "PROJECT " << i << " " << j << " " << k << " " << std::endl;

//        const std::set<unsigned> & elemntsID = geometry()->getIndexedElements(i,j,k);
//        double min_dist = std::numeric_limits<double>::max();
//        ConstraintProximityPtr res = NULL;
//        //search for the closest triangle from the center of the AABB
//        for (std::set<unsigned>::iterator t=elemntsID.begin();t!=elemntsID.end();t++) {
//            ConstraintElementPtr edest = geometry()->p_geometry->getElement(*t);
//            ConstraintProximityPtr pdest = edest->project(P);

//            double dist = pdest->distance(P);
//            if (dist<min_dist) {
//                min_dist = dist;
//                res = pdest;
//            }
//        }
        return m_controlPoint;
    }

    inline AABBGeometry * geometry() const {
        return (AABBGeometry*) m_geometry;
    }

    void draw(const VisualParams * /*vparams*/) {
        Vector3 points[8];

        points[0] = geometry()->m_Bmin + Vector3((i  ) * geometry()->m_cellSize[0],(j  ) * geometry()->m_cellSize[1],(k  ) * geometry()->m_cellSize[2]) ;
        points[1] = geometry()->m_Bmin + Vector3((i+1) * geometry()->m_cellSize[0],(j  ) * geometry()->m_cellSize[1],(k  ) * geometry()->m_cellSize[2]) ;
        points[2] = geometry()->m_Bmin + Vector3((i  ) * geometry()->m_cellSize[0],(j+1) * geometry()->m_cellSize[1],(k  ) * geometry()->m_cellSize[2]) ;
        points[3] = geometry()->m_Bmin + Vector3((i+1) * geometry()->m_cellSize[0],(j+1) * geometry()->m_cellSize[1],(k  ) * geometry()->m_cellSize[2]) ;
        points[4] = geometry()->m_Bmin + Vector3((i  ) * geometry()->m_cellSize[0],(j  ) * geometry()->m_cellSize[1],(k+1) * geometry()->m_cellSize[2]) ;
        points[5] = geometry()->m_Bmin + Vector3((i+1) * geometry()->m_cellSize[0],(j  ) * geometry()->m_cellSize[1],(k+1) * geometry()->m_cellSize[2]) ;
        points[6] = geometry()->m_Bmin + Vector3((i  ) * geometry()->m_cellSize[0],(j+1) * geometry()->m_cellSize[1],(k+1) * geometry()->m_cellSize[2]) ;
        points[7] = geometry()->m_Bmin + Vector3((i+1) * geometry()->m_cellSize[0],(j+1) * geometry()->m_cellSize[1],(k+1) * geometry()->m_cellSize[2]) ;

        glColor4f(1,0,0,1);
        glBegin(GL_LINES);
            glVertex3dv(points[0].data());glVertex3dv(points[1].data());
            glVertex3dv(points[3].data());glVertex3dv(points[2].data());
            glVertex3dv(points[7].data());glVertex3dv(points[6].data());
            glVertex3dv(points[4].data());glVertex3dv(points[5].data());

            glVertex3dv(points[0].data());glVertex3dv(points[2].data());
            glVertex3dv(points[1].data());glVertex3dv(points[3].data());
            glVertex3dv(points[4].data());glVertex3dv(points[6].data());
            glVertex3dv(points[5].data());glVertex3dv(points[7].data());

            glVertex3dv(points[0].data());glVertex3dv(points[4].data());
            glVertex3dv(points[1].data());glVertex3dv(points[5].data());
            glVertex3dv(points[2].data());glVertex3dv(points[6].data());
            glVertex3dv(points[3].data());glVertex3dv(points[7].data());
        glEnd();
    }


private:
    const unsigned i,j,k;
    ConstraintProximityPtr m_controlPoint;
};

}
