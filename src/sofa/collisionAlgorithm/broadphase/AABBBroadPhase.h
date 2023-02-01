#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class AABBBroadPhase : public BaseGeometry::BroadPhase {
public:

    SOFA_CLASS(AABBBroadPhase,BaseGeometry::BroadPhase);

    Data<type::Vec3i> d_nbox;
    Data<bool> d_static;

    core::objectmodel::DataCallback c_nbox;

    AABBBroadPhase()
    : d_nbox(initData(&d_nbox, type::Vec3i(8,8,8),"nbox", "number of bbox"))
    , d_static(initData(&d_static, false,"isStatic", "Optimization: object is not moving in the scene")) {
        c_nbox.addInputs({&d_nbox});
        c_nbox.addCallback(std::bind(&AABBBroadPhase::updateBroadPhase,this));
    }

    type::BoundingBox getBBox() const {
        return type::BoundingBox(m_Bmin,m_Bmax);
    }

    const std::set<BaseElement::SPtr> & getElementSet(unsigned i, unsigned j, unsigned k) const override {
        auto it = m_indexedElement.find(getKey(i,j,k));
        if (it == m_indexedElement.end()) {
            static std::set<BaseElement::SPtr> empty;
            return empty;
        } else {
            return it->second;
        }
    }

    void initBroadPhase() override {
        doUpdate();
    }

    void updateBroadPhase() override {
        if(d_static.getValue()) return;
        doUpdate();
    }

    inline void doUpdate() {
        sofa::helper::AdvancedTimer::stepBegin("========================= AABBBroadPhase do update =========================");
        m_Bmin = l_geometry->getPosition(0);
        m_Bmax = m_Bmin;

        //updates bounding box area
        sofa::helper::AdvancedTimer::stepBegin("========================= BBox area update in AABBBroadPhase do update =========================");
        for (unsigned j=1;j<l_geometry->getSize();j++) {
            type::Vector3 pos = l_geometry->getPosition(j);

            for (int i = 0 ; i < 3 ; i++) {
                if (pos[i] > m_Bmax[i])
                    m_Bmax[i] = pos[i] ;
                if (pos[i] < m_Bmin[i])
                    m_Bmin[i] = pos[i] ;
            }

    //        if (pos[0]<m_Bmin[0]) m_Bmin[0] = pos[0];
    //        if (pos[1]<m_Bmin[1]) m_Bmin[1] = pos[1];
    //        if (pos[2]<m_Bmin[2]) m_Bmin[2] = pos[2];

    //        if (pos[0]>m_Bmax[0]) m_Bmax[0] = pos[0];
    //        if (pos[1]>m_Bmax[1]) m_Bmax[1] = pos[1];
    //        if (pos[2]>m_Bmax[2]) m_Bmax[2] = pos[2];
        }
        sofa::helper::AdvancedTimer::stepEnd("========================= BBox area update in AABBBroadPhase do update =========================");

        //fixes cell size
        for (int i = 0 ; i < 3 ; i++) {
            m_cellSize[i] = (m_Bmax[i] - m_Bmin[i]) / d_nbox.getValue()[i];
        }
    //    m_cellSize[0] = (m_Bmax[0] - m_Bmin[0]) / d_nbox.getValue()[0];
    //    m_cellSize[1] = (m_Bmax[1] - m_Bmin[1]) / d_nbox.getValue()[1];
    //    m_cellSize[2] = (m_Bmax[2] - m_Bmin[2]) / d_nbox.getValue()[2];



        for (int i = 0 ; i < 3 ; i++) {
            if (m_cellSize[i] == 0) {
                int a = (i == 0) ? 1 : 0 ;
                int b = (i == 2) ? 1 : 2 ;
                m_cellSize[i] = (m_cellSize[a]+m_cellSize[b])*0.5;
                m_nbox[i] = 1;
            } else {
                m_nbox[i] = d_nbox.getValue()[i] + 1;
            }
        }

    //    if (m_cellSize[0] == 0) {
    //        m_cellSize[0] = (m_cellSize[1]+m_cellSize[2])*0.5;
    //        m_nbox[0] = 1;
    //    }
    //    else {
    //        m_nbox[0] = d_nbox.getValue()[0] + 1;
    //    }

    //    if (m_cellSize[1] == 0) {
    //        m_cellSize[1] = (m_cellSize[0]+m_cellSize[2])*0.5;
    //        m_nbox[1] = 1;
    //    }
    //    else
    //        m_nbox[1] = d_nbox.getValue()[1] + 1;

    //    if (m_cellSize[2] == 0)
    //    {
    //        m_cellSize[2] = (m_cellSize[0]+m_cellSize[1])*0.5;
    //        m_nbox[2] = 1;
    //    }
    //    else
    //        m_nbox[2] = d_nbox.getValue()[2] + 1;

        sofa::helper::AdvancedTimer::stepBegin("========================= CLEAR MAP =========================");
        m_indexedElement.clear();
        sofa::helper::AdvancedTimer::stepEnd("========================= CLEAR MAP =========================");

        m_offset[0] = m_nbox[1]*m_nbox[2];
        m_offset[1] = m_nbox[2];

        // center in -0.5 cellwidth
        m_Bmin -= m_cellSize * 0.5;
        m_Bmax -= m_cellSize * 0.5;


        sofa::helper::AdvancedTimer::stepBegin("========================= Elements rangés dans boites in AABB doUpdate =========================");
//        updateElemInBoxes();

        auto projectOp = Operations::Project::Operation::get(l_geometry);

        for (auto it = l_geometry->begin(); it != l_geometry->end(); it++)
        {
            BaseElement::SPtr elmt = it->element();

            //std::cout << ++i << std::endl;
            type::BoundingBox bbox;
//            std::cout << "size v_prox : " << v_prox.size() << std::endl;

            for (auto it = elmt->pointElements().cbegin(); it!= elmt->pointElements().cend(); it++) {
                bbox.include((*it)->getP0()->getPosition());
            }

            const type::Vector3 & minbox = bbox.minBBox();
            const type::Vector3 & maxbox = bbox.maxBBox();

            type::Vec3i cminbox(0,0,0);
            type::Vec3i cmaxbox(0,0,0);

            for (int i = 0 ; i < 3 ; i++) {
                cmaxbox[i] = ceil((maxbox[i] - m_Bmin[i])/m_cellSize[i]);
                cminbox[i] = floor((minbox[i] - m_Bmin[i])/m_cellSize[i]); //second m_Bmax was Bmin => bug ?
            }

            for (int i=cminbox[0];i<cmaxbox[0];i++)
            {
                unsigned key_i = i*m_offset[0];

                for (int j=cminbox[1];j<cmaxbox[1];j++)
                {
                    unsigned key_j = j*m_offset[1];

                    for (int k=cminbox[2];k<cmaxbox[2];k++)
                    {
                        unsigned key = key_i + key_j + k;

                        type::Vector3 P = m_Bmin + m_cellSize*0.5;

                        P[0] += i*m_cellSize[0];
                        P[1] += j*m_cellSize[1];
                        P[2] += k*m_cellSize[2];

                        sofa::helper::AdvancedTimer::stepBegin("========================= Project from AABB doUpdate =========================");
                        BaseProximity::SPtr prox = projectOp(P,elmt).prox;
                        sofa::helper::AdvancedTimer::stepEnd("========================= Project from AABB doUpdate =========================");
                        if (prox == NULL) continue;

                        prox->normalize();
                        type::Vector3 D = prox->getPosition()-P;

                        if ((fabs(D[0])<=m_cellSize[0]*0.6) &&
                            (fabs(D[1])<=m_cellSize[1]*0.6) &&
                            (fabs(D[2])<=m_cellSize[2]*0.6)) {

                            sofa::helper::AdvancedTimer::stepBegin("========================= Adding elem asociated to key in MAP =========================");
                            m_indexedElement[key].insert(elmt);
                            sofa::helper::AdvancedTimer::stepEnd("========================= Adding elem asociated to key in MAP =========================");
                        }
                    }
                }
            }
        }
        sofa::helper::AdvancedTimer::stepEnd("========================= Elements rangés dans boites in AABB doUpdate =========================");
        sofa::helper::AdvancedTimer::stepEnd("========================= AABBBroadPhase do update =========================");
    }

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowBoundingCollisionModels()) return;

        type::RGBAColor color(1,0,0,1);

        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        glColor4f(color[0],color[1],color[2],color[3]);
        for (auto it = m_indexedElement.cbegin();it != m_indexedElement.cend(); it++) {
            unsigned i = getIKey(it->first);
            unsigned j = getJKey(it->first);
            unsigned k = getKKey(it->first);

            type::Vector3 min = m_Bmin + type::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
            type::Vector3 max = m_Bmin + type::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
            type::BoundingBox bbox(min,max);

            type::Vector3 points[8];

            points[0] = type::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
            points[1] = type::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
            points[2] = type::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
            points[3] = type::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
            points[4] = type::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
            points[5] = type::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
            points[6] = type::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);
            points[7] = type::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);


            if (vparams->displayFlags().getShowWireFrame()) {
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
            } else {
                glBegin(GL_QUADS);
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

    inline Index getKey(size_t i,size_t j,size_t k) const {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    inline const type::Vector3 & getMin() const {
        return m_Bmin;
    }

    inline const type::Vector3 & getMax() const {
        return m_Bmax;
    }

    inline const type::Vector3 & getCellSize() const {
        return m_cellSize;
    }

    //compute the box where is P
    type::Vec3i getBoxCoord(const type::Vector3 & P) const override {
        return type::Vec3i((P[0] - m_Bmin[0])/m_cellSize[0],
                           (P[1] - m_Bmin[1])/m_cellSize[1],
                           (P[2] - m_Bmin[2])/m_cellSize[2]);
    }

    inline unsigned getIKey(unsigned key) {
        return key/m_offset[0];
    }

    inline unsigned getJKey(unsigned key) {
        return (key - getIKey(key) * m_offset[0])/m_offset[1];
    }

    inline unsigned getKKey(unsigned key) {
        return key - getIKey(key) * m_offset[0] - getJKey(key) * m_offset[1];
    }

    type::Vec3i getNbox() override {
        return d_nbox.getValue();
    }

    void updateElemInBoxes() {
        for (auto it = l_geometry->begin(); it != l_geometry->end(); it++) {
            sofa::helper::AdvancedTimer::stepBegin("========================= Geometry elements visited in updatElemInBoxes =========================");
            BaseElement::SPtr elmt = it->element();

            std::set<unsigned int> boxKey;
            bool sameBox = 1;
            for (auto it = elmt->pointElements().cbegin(); it!= elmt->pointElements().cend(); it++) {
                type::Vec3i boxCoord = getBoxCoord((*it)->getP0()->getPosition());
                boxKey.insert(getKey(boxCoord[0],boxCoord[1],boxCoord[2]));
            }

            // If all the points of the element are located in the same cell
            if (boxKey.size() == 1) {
                sofa::helper::AdvancedTimer::stepBegin("========================= INSERT IN MAP IN 1 SINGLE CELL =========================");
                unsigned key = *boxKey.begin();
                m_indexedElement[key].insert(elmt);
                sofa::helper::AdvancedTimer::stepEnd("========================= INSERT IN MAP IN 1 SINGLE CELL =========================");
            }

            // Otherwise, test triangle-cell overlap between the element and the cells contained in its bounding box
            else {
                sofa::helper::AdvancedTimer::stepBegin("========================= INSERT IN MAP IN SEVERAL CELLS FROM POINTS =========================");
                std::set<unsigned int>::iterator it;
                for (it=boxKey.begin(); it!=boxKey.end(); it++) {
                    unsigned key = *it;
                    m_indexedElement[key].insert(elmt);
                }
                sofa::helper::AdvancedTimer::stepEnd("========================= INSERT IN MAP IN SEVERAL CELLS FROM POINTS =========================");
                sofa::helper::AdvancedTimer::stepBegin("========================= multipleCells test =========================");
                multipleCells(elmt,boxKey);
                sofa::helper::AdvancedTimer::stepEnd("========================= multipleCells test =========================");
            }
            sofa::helper::AdvancedTimer::stepEnd("========================= Geometry elements visited in updatElemInBoxes =========================");
        }
    }


    // /// Based on Akenine-Möller, T. (2001). "Fast 3D Triangle-Box Overlap Testing".
    // /// and      https://gist.github.com/yomotsu/d845f21e2e1eb49f647f
    void multipleCells(BaseElement::SPtr elmt, std::set<unsigned int> & boxKey) {
        type::BoundingBox bbox;

        for (auto it = elmt->pointElements().cbegin(); it!= elmt->pointElements().cend(); it++) {
            bbox.include((*it)->getP0()->getPosition());
        }

        const type::Vector3 & minbox = bbox.minBBox();
        const type::Vector3 & maxbox = bbox.maxBBox();

        type::Vec3i cminbox(0,0,0);
        type::Vec3i cmaxbox(0,0,0);

        for (int i = 0 ; i < 3 ; i++) {
            cmaxbox[i] = ceil((maxbox[i] - m_Bmin[i])/m_cellSize[i]);
            cminbox[i] = floor((minbox[i] - m_Bmin[i])/m_cellSize[i]); //second m_Bmax was Bmin => bug ?
        }



        for (int i=cminbox[0];i<cmaxbox[0];i++)
        {
            unsigned key_i = i*m_offset[0];

            for (int j=cminbox[1];j<cmaxbox[1];j++)
            {
                unsigned key_j = j*m_offset[1];

                for (int k=cminbox[2];k<cmaxbox[2];k++)
                {
                    unsigned key = key_i + key_j + k;
                    if (std::find(boxKey.begin(), boxKey.end(), key) != boxKey.end()) //no need to process this cell if it has already been registered from the vertices
                        continue;

                    type::Vec3d bMaxCell = m_Bmin;
                    bMaxCell[0] += (i+1)*m_cellSize[0];
                    bMaxCell[1] += (j+1)*m_cellSize[1];
                    bMaxCell[2] += (k+1)*m_cellSize[2];
                    type::Vec3d cellCenter = m_Bmin + m_cellSize*0.5;
                    cellCenter[0] += i*m_cellSize[0];
                    cellCenter[1] += j*m_cellSize[1];
                    cellCenter[2] += k*m_cellSize[2];
                    type::Vec3d extents = bMaxCell - cellCenter;

                    for (auto it = elmt->triangleElements().cbegin(); it!= elmt->triangleElements().cend(); it++)
                    {
                        // Axes related to the cell's normals
                        type::Vec3d e0(1,0,0);
                        type::Vec3d e1(0,1,0);
                        type::Vec3d e2(0,0,1);

                        type::Vec3d v0 = (*it)->getP0()->getPosition() - cellCenter;
                        type::Vec3d v1 = (*it)->getP1()->getPosition() - cellCenter;
                        type::Vec3d v2 = (*it)->getP2()->getPosition() - cellCenter;

                        type::Vec3d f0 = v1 - v0;
                        type::Vec3d f1 = v2 - v1;
                        type::Vec3d f2 = v0 - v2;
                        std::vector<type::Vec3d> f = {f0,f1,f2};

                        // Axes orthogonal to triangle's edges
                        type::Vec3d a00 = cross(e0,f0);
                        type::Vec3d a01 = cross(e0,f1);
                        type::Vec3d a02 = cross(e0,f2);
                        type::Vec3d a10 = cross(e1,f0);
                        type::Vec3d a11 = cross(e1,f1);
                        type::Vec3d a12 = cross(e1,f2);
                        type::Vec3d a20 = cross(e2,f0);
                        type::Vec3d a21 = cross(e2,f1);
                        type::Vec3d a22 = cross(e2,f2);
                        std::vector<std::vector<type::Vec3d>> a = {{a00,a01,a02},
                                                                   {a10,a11,a12},
                                                                   {a20,a21,a22}};


                        // /// 3 tests, for 3 categories of axes /// //
                        // Test1
                        bool test_CellNormals = testCellNormals(v0,v1,v2,extents);
                        if (!test_CellNormals) continue; // an axis provides no overlap --> this triangle does not intersect the cell, test another one

                        // Test 2
                        type::Vec3d triNormal = cross(f1,f0).normalized();
                        double constant = dot(triNormal,(*it)->getP0()->getPosition());
                        double r = extents[0]*std::abs(triNormal(0)) + extents[1]*std::abs(triNormal[1]) + extents[2]*std::abs(triNormal[2]);
                        double center2planeDist = dot(triNormal,cellCenter) - constant;
                        bool test_triangleNormal = (std::abs(center2planeDist) <= r); // true in case of an overlap, false otherwise
                        if (!test_triangleNormal) continue;

                        // Test 3
                        bool test_CrossProdAxes = testCrossProdAxes(a,extents,f,v0,v1,v2);
                        if (!test_CrossProdAxes) continue;


                        // We get here iff all tests indicated an overlap between the triangle and the cell (ie: all tests returned true)
                        sofa::helper::AdvancedTimer::stepBegin("========================= INSERT IN MAP AFTER ALL THE TESTS =========================");
                        m_indexedElement[key].insert(elmt);
                        sofa::helper::AdvancedTimer::stepEnd("========================= INSERT IN MAP AFTER ALL THE TESTS =========================");
                        break;
                    }
                }
            }
        }
    }

    bool testCellNormals(type::Vec3d & v0, type::Vec3d & v1, type::Vec3d & v2, type::Vec3d & extents) {
        if (std::max(std::max(v0[0],v1[0]),v2[0])<-extents[0] || std::min(std::min(v0[0],v1[0]),v2[0])>extents[0]) return false;

        if (std::max(std::max(v0[1],v1[1]),v2[1])<-extents[1] || std::min(std::min(v0[1],v1[1]),v2[1])>extents[1]) return false;

        if (std::max(std::max(v0[2],v1[2]),v2[2])<-extents[2] || std::min(std::min(v0[2],v1[2]),v2[2])>extents[2]) return false;

        return true;
    }


    bool testCrossProdAxes(std::vector<std::vector<type::Vec3d>> & a,
                           type::Vec3d & extents,
                           std::vector<type::Vec3d> & f,
                           type::Vec3d & v0, type::Vec3d & v1, type::Vec3d & v2)
    {
        unsigned idx_1, idx_2;
        bool test_crossProd;
        for (unsigned j=0; j<a.size(); j++) {
            for (unsigned i=0; i<a[j].size(); i++) {
                idx_1 = (j != 0) ? 0 : 1;
                idx_2 = (j != 2) ? 2 : 1;
                test_crossProd = testCrossProdSingleAxe(extents[idx_1], extents[idx_2],
                                                        f[i][idx_2], f[i][idx_1],
                                                        a[j][i],
                                                        v0, v1, v2);
                if (!test_crossProd) return false;
            }
        }
        return true;
    }


    bool testCrossProdSingleAxe(double extents0, double extents1,
                                double f0, double f1,
                                type::Vec3d & a,
                                type::Vec3d & v0, type::Vec3d & v1, type::Vec3d & v2)
    {
        double p0 = dot(v0,a);
        double p1 = dot(v1,a);
        double p2 = dot(v2,a);

        double r = extents0*std::abs(f0) + extents1*std::abs(f1);

        if (std::max(-std::max(std::max(p0,p1),p2), std::min(std::min(p0,p1),p2)) > r) return false; // no overlap detected

        return true;
    }


protected:
    type::Vector3 m_Bmin,m_Bmax,m_cellSize;
    type::Vec3i m_nbox;
    type::Vec<2, size_t> m_offset;
    std::map<unsigned, std::set<BaseElement::SPtr> > m_indexedElement;
};


}
