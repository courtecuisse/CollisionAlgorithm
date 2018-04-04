#pragma once

#include <geometry/AABBGeometry.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

//class AABBProximity : public BaseGeometry::ConstraintProximity {
//public :
//    AABBProximity(const AABBProximity * geo) {
//        m_geo = geo;
//    }

//    Vector3 getPosition(core::VecCoordId vid) const {
//        return Vector3();
//    }

//    void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const Vector3 & N) {

//    }
//};

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

AABBElement::AABBElement(AABBGeometry * geo)
: ConstraintElement(geo,0) {}

//this function returns a vector with all the control points of the element
ConstraintProximityPtr AABBElement::getControlPoint(const int /*cid*/) {
    return NULL;
}

void AABBElement::fillElementSet(Vec3i cbox, int d, std::set<int> & selectElements) {
    for (int i=-d;i<=d;i++) {
        if (cbox[0]+i < 0 || cbox[0]+i > geometry()->d_nbox.getValue()[0]) continue;
        if (i!=-d && i!=d) continue;

        for (int j=-d;j<=d;j++) {
            if (cbox[1]+j < 0 || cbox[1]+j > geometry()->d_nbox.getValue()[1]) continue;
            if (j!=-d && j!=d) continue;

            for (int k=-d;k<=d;k++) {
                if (cbox[2]+k < 0 || cbox[2]+k > geometry()->d_nbox.getValue()[2]) continue;
                if (k!=-d && k!=d) continue;

                const std::vector<unsigned> & elemntsID = geometry()->m_indexedElement[cbox[0] + i][cbox[1] + j][cbox[2] + k];
                for (unsigned t=0;t<elemntsID.size();t++) selectElements.insert(elemntsID[t]);
            }
        }
    }
}

//this function project the point P on the element and return the corresponding proximity
ConstraintProximityPtr AABBElement::project(Vector3 P) {
    //compute the box where is P
    Vec3i cbox;
    cbox[0] = floor((P[0] - geometry()->m_Bmin[0])/geometry()->m_cellSize[0]);
    cbox[1] = floor((P[1] - geometry()->m_Bmin[1])/geometry()->m_cellSize[1]);
    cbox[2] = floor((P[2] - geometry()->m_Bmin[2])/geometry()->m_cellSize[2]);

    //project the box in the bounding box of the object
    //search with the closest box in bbox
    for (int i=0;i<3;i++) {
        if (cbox[i] < 0) cbox[i] = 0;
        else if (cbox[i] > geometry()->d_nbox.getValue()[i]) cbox[i] = geometry()->d_nbox.getValue()[i];
    }

    int d = 0;
    int max = std::max(std::max(geometry()->d_nbox.getValue()[0],geometry()->d_nbox.getValue()[1]),geometry()->d_nbox.getValue()[2]);
    std::set<int> selectElements;
    while (selectElements.empty() && d<max) {
        fillElementSet(cbox,d,selectElements);
        d++;// we look for boxed located at d+1
    }

    ConstraintProximityPtr res = NULL;
    double min_norm = std::numeric_limits<double>::max();
    for (std::set<int>::iterator it = selectElements.begin();it!=selectElements.end();it++) {
        ConstraintProximityPtr tmp_res = geometry()->p_geometry->getElement(*it)->project(P);

        double norm = (tmp_res->getPosition() - P).norm();

        if (norm<min_norm) {
            res = tmp_res;
            min_norm = norm;
        }
    }

    return res;
}

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

AABBGeometry::AABBGeometry()
: d_nbox("nbox",Vec3i(8,8,8),this)
, p_geometry(this)
{}

void AABBGeometry::init() {
    m_elements.clear();
    m_elements.push_back(std::make_shared<AABBElement>(this));
}

void AABBGeometry::prepareDetection() {
    m_indexedElement.resize(d_nbox.getValue()[0]+1);
    for (unsigned i=0;i<m_indexedElement.size();i++) {
        m_indexedElement[i].resize(d_nbox.getValue()[1]+1);
        for (unsigned j=0;j<m_indexedElement[i].size();j++) {
            m_indexedElement[i][j].resize(d_nbox.getValue()[2]+1);

            for (unsigned k=0;k<m_indexedElement[i][j].size();k++) {
                m_indexedElement[i][j][k].clear();
            }
        }
    }

    BoundingBox bbox;
    const ReadAccessor<Vector3> & pos = p_topology->p_state->read(VecCoordId::position());
    for (unsigned i=0;i<pos.size();i++) bbox.include(pos[i]);

    m_Bmin = Vector3(bbox.minBBoxPtr());
    m_Bmax = Vector3(bbox.minBBoxPtr());

    m_cellSize[0] = (m_Bmax[0] - m_Bmin[0]) / d_nbox.getValue()[0];
    m_cellSize[1] = (m_Bmax[1] - m_Bmin[1]) / d_nbox.getValue()[1];
    m_cellSize[2] = (m_Bmax[2] - m_Bmin[2]) / d_nbox.getValue()[2];

    // center in -0.5 cellwidth
    m_Bmin -= m_cellSize * 0.5;
    m_Bmax -= m_cellSize * 0.5;

    for (unsigned itE = 0; itE < p_geometry->getNbElements(); itE++) {
        ConstraintElementPtr elmt = p_geometry->getElement(itE);
        if (elmt->getNbControlPoints() == 0) continue;

        Vector3 minbox(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
        Vector3 maxbox(std::numeric_limits<double>::min(),std::numeric_limits<double>::min(),std::numeric_limits<double>::min());

        for (unsigned p=0;p<elmt->getNbControlPoints();p++) {
            Vector3 P = elmt->getControlPoint(p)->getPosition();

            minbox[0] = std::min(minbox[0],P[0]);
            minbox[1] = std::min(minbox[1],P[1]);
            minbox[2] = std::min(minbox[2],P[2]);

            maxbox[0] = std::max(maxbox[0],P[0]);
            maxbox[1] = std::max(maxbox[1],P[1]);
            maxbox[2] = std::max(maxbox[2],P[2]);
        }

        Vec3i cminbox;
        Vec3i cmaxbox;

        cminbox[0] = floor((minbox[0] - m_Bmin[0])/m_cellSize[0]);
        cminbox[1] = floor((minbox[1] - m_Bmin[1])/m_cellSize[1]);
        cminbox[2] = floor((minbox[2] - m_Bmin[2])/m_cellSize[2]);

        cmaxbox[0] = ceil((maxbox[0] - m_Bmin[0])/m_cellSize[0]);
        cmaxbox[1] = ceil((maxbox[1] - m_Bmin[1])/m_cellSize[1]);
        cmaxbox[2] = ceil((maxbox[2] - m_Bmin[2])/m_cellSize[2]);

        for (int i=cminbox[0];i<cmaxbox[0];i++) {
            for (int j=cminbox[1];j<cmaxbox[1];j++) {
                for (int k=cminbox[2];k<cmaxbox[2];k++) {
                    m_indexedElement[i][j][k].push_back(itE);
                }
            }
        }
    }
}

void AABBGeometry::draw(const VisualParams * vparams) {
    if (!vparams->displayFlags().getShowCollisionModels()) return;

    for (unsigned i=0;i<m_indexedElement.size();i++) {
        for (unsigned j=0;j<m_indexedElement[i].size();j++) {
            for (unsigned k=0;k<m_indexedElement[i][j].size();k++) {
                if (m_indexedElement[i][j][k].empty()) continue;

                Vector3 points[8];
                points[0] = m_Bmin + Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[1] = m_Bmin + Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[2] = m_Bmin + Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[3] = m_Bmin + Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[4] = m_Bmin + Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[5] = m_Bmin + Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[6] = m_Bmin + Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[7] = m_Bmin + Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;

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
        }
    }
}

}
