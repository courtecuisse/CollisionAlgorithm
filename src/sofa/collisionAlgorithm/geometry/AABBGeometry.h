#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa::collisionAlgorithm {

class AABBBElement : public BaseElement {
public:

    typedef std::shared_ptr<AABBBElement> SPtr;

    AABBBElement(unsigned  eid, unsigned i,unsigned j, unsigned k, const std::vector<BaseProximity::SPtr> & elmts)
    : m_eid(eid)
    , m_i(i)
    , m_j(j)
    , m_k(k)
    , m_projProx(elmts) {}


    virtual unsigned id() override {
        return m_eid;
    }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res = m_projProx;
    }

    unsigned i() const {return m_i;}

    unsigned j() const {return m_j;}

    unsigned k() const {return m_k;}

private:
    unsigned m_eid,m_i,m_j,m_k;
    std::vector<BaseProximity::SPtr> m_projProx;
};

class AABBGeometry : public BaseGeometry {
public:

    SOFA_CLASS(AABBGeometry,BaseGeometry);

    typedef BaseGeometry::BaseGeometry::Index Index;

    Data<type::Vec3i> d_nbox;
    Data<bool> d_static;

    core::objectmodel::SingleLink<AABBGeometry, BaseGeometry, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    AABBGeometry()
    : d_nbox(initData(&d_nbox, type::Vec3i(8,8,8),"nbox", "number of bbox"))
    , d_static(initData(&d_static, false,"isStatic", "Optimization: object is not moving in the scene"))
    , l_geometry(initLink("geometry", "link to geometry"))
    , m_staticInitDone(false) {
        l_geometry.setPath("@.");
    }

    ElementIterator::SPtr begin() const override {
        return ElementIterator::SPtr(new DefaultElementIterator(this));
    }

    unsigned elementSize() const override {
        return l_geometry->elementSize();
    }

    BaseElement::SPtr getElement(unsigned i) const override {
        return l_geometry->getElement(i);
    }

    virtual size_t getOperationsHash() const {
        return typeid(AABBGeometry).hash_code();
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const {
        return l_geometry->getState();
    }

    type::BoundingBox getBBox() const {
        return type::BoundingBox(m_Bmin,m_Bmax);
    }

    void init() {
        prepareDetection();
    }

    void getElementSet(type::Vec3i c, std::set<Index> & selectElements) const {
//        auto it = m_indexedElement.find(getKey(c[0],c[1],c[2]));
//        if (it != m_indexedElement.end()) {
//            const std::set<Index> & elemntsID = it->second;
//            selectElements.insert(elemntsID.begin(),elemntsID.end());
//        }
    }

    void prepareDetection() override {
        if (l_geometry == NULL) return;

        if(d_static.getValue() && m_staticInitDone) return;

        m_staticInitDone = true;

        sofa::core::behavior::BaseMechanicalState * mstate = l_geometry->getState();

        m_Bmin = type::Vector3(mstate->getPX(0),mstate->getPY(0),mstate->getPZ(0));
        m_Bmax = m_Bmin;

        //updates bounding box area
        for (unsigned j=1;j<mstate->getSize();j++) {
            type::Vector3 pos(mstate->getPX(j),mstate->getPY(j),mstate->getPZ(j));

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

        std::map<Index, std::vector<BaseProximity::SPtr> > indexedElement;
        m_offset[0] = m_nbox[1]*m_nbox[2];
        m_offset[1] = m_nbox[2];

        // center in -0.5 cellwidth
        m_Bmin -= m_cellSize * 0.5;
        m_Bmax -= m_cellSize * 0.5;

        auto projectOp = Operations::ProjectOperation::func(l_geometry.get());

        for (auto it = l_geometry->begin(); it != l_geometry->end(); it++)
        {
            BaseElement::SPtr elmt = it->element();

            //std::cout << ++i << std::endl;
            type::BoundingBox bbox;
            std::vector<BaseProximity::SPtr> v_prox;
            elmt->getControlProximities(v_prox);

            for (unsigned b=0;b<v_prox.size();b++) {
                bbox.include(v_prox[b]->getPosition());
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

                        BaseProximity::SPtr prox = projectOp(P,elmt);
                        if (prox == NULL) continue;

                        type::Vector3 D = prox->getPosition()-P;

                        if ((fabs(D[0])<=m_cellSize[0]*0.5) &&
                            (fabs(D[1])<=m_cellSize[1]*0.5) &&
                            (fabs(D[2])<=m_cellSize[2]*0.5))
                            indexedElement[key].push_back(prox);
                    }
                }
            }
        }



//        for (auto it = indexedElement.begin();it!=indexedElement.end();it++) {
//            unsigned eid = it->first;

//            unsigned i = (eid) / m_offset[0];
//            unsigned j = (eid - i*m_offset[0]) / m_offset[1];
//            unsigned k = (eid - i*m_offset[0] - j*m_offset[1]);

//            m_elements.push_back(AABBBElement::SPtr(new AABBBElement(eid,i,j,k,it->second)));
//        }
    }

    void draw(const core::visual::VisualParams * vparams) {
//        auto projectOp = Operations::Project::func(l_geometry);
//        for (auto it = l_geometry->begin(); it != l_geometry->end(); it++)
//        {
//            BaseElement::SPtr elmt = it->element();

//            //std::cout << ++i << std::endl;
//            type::BoundingBox bbox;
//            std::vector<BaseProximity::SPtr> v_prox;
//            elmt->getControlProximities(v_prox);

//            for (unsigned b=0;b<v_prox.size();b++) {
//                bbox.include(v_prox[b]->getPosition());
//            }

//            const type::Vector3 & minbox = bbox.minBBox();
//            const type::Vector3 & maxbox = bbox.maxBBox();

//            type::Vec3i cminbox(0,0,0);
//            type::Vec3i cmaxbox(0,0,0);

//            for (int i = 0 ; i < 3 ; i++) {
//                cmaxbox[i] = ceil((maxbox[i] - m_Bmin[i])/m_cellSize[i]);
//                cminbox[i] = floor((minbox[i] - m_Bmin[i])/m_cellSize[i]); //second m_Bmax was Bmin => bug ?
//            }

//            for (int i=cminbox[0];i<cmaxbox[0];i++)
//            {
//                for (int j=cminbox[1];j<cmaxbox[1];j++)
//                {
//                    for (int k=cminbox[2];k<cmaxbox[2];k++)
//                    {
//                        type::Vector3 P = m_Bmin + m_cellSize*0.5;

//                        P[0] += i*m_cellSize[0];
//                        P[1] += j*m_cellSize[1];
//                        P[2] += k*m_cellSize[2];

//                        BaseProximity::SPtr prox = projectOp(P,elmt);
//                        if (prox == NULL) continue;

//                        type::Vector3 D = prox->getPosition();

//                        glBegin(GL_LINES);
//                            glColor3f(1,0,0);
//                            glVertex3dv(P.data());
//                            glColor3f(0,1,0);
//                            glVertex3dv(D.data());
//                        glEnd();

//                    }
//                }
//            }
//        }


        if (! vparams->displayFlags().getShowBoundingCollisionModels()) return;

        if (l_geometry == NULL) return;

        if (this->l_geometry->d_color.getValue().a() == 0.0) return;

        glDisable(GL_LIGHTING);

        glColor4fv(this->l_geometry->d_color.getValue().data());

//        for (unsigned e=0;e<m_elements.size();e++) {
//            unsigned i = m_elements[e]->i();
//            unsigned j = m_elements[e]->j();
//            unsigned k = m_elements[e]->k();

//            type::Vector3 min = m_Bmin + type::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
//            type::Vector3 max = m_Bmin + type::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
//            type::BoundingBox bbox(min,max);

//            type::Vector3 points[8];

//            points[0] = type::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
//            points[1] = type::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
//            points[2] = type::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
//            points[3] = type::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
//            points[4] = type::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
//            points[5] = type::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
//            points[6] = type::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);
//            points[7] = type::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);


//    //        if (vparams->displayFlags().getShowWireFrame()) {
//                glBegin(GL_LINES);
//                    glVertex3dv(points[0].data());glVertex3dv(points[1].data());
//                    glVertex3dv(points[3].data());glVertex3dv(points[2].data());
//                    glVertex3dv(points[7].data());glVertex3dv(points[6].data());
//                    glVertex3dv(points[4].data());glVertex3dv(points[5].data());

//                    glVertex3dv(points[0].data());glVertex3dv(points[2].data());
//                    glVertex3dv(points[1].data());glVertex3dv(points[3].data());
//                    glVertex3dv(points[4].data());glVertex3dv(points[6].data());
//                    glVertex3dv(points[5].data());glVertex3dv(points[7].data());

//                    glVertex3dv(points[0].data());glVertex3dv(points[4].data());
//                    glVertex3dv(points[1].data());glVertex3dv(points[5].data());
//                    glVertex3dv(points[2].data());glVertex3dv(points[6].data());
//                    glVertex3dv(points[3].data());glVertex3dv(points[7].data());
//                glEnd();
//    //        } else {
//    //            glBegin(GL_QUADS);
//    //                glColor3dv((this->d_color.getValue()*0.8).data());
//    //                glVertex3dv(points[0].data());glVertex3dv(points[1].data());glVertex3dv(points[3].data());glVertex3dv(points[2].data());

//    //                glColor3dv((this->d_color.getValue()*0.7).data());
//    //                glVertex3dv(points[4].data());glVertex3dv(points[5].data());glVertex3dv(points[7].data());glVertex3dv(points[6].data());

//    //                glColor3dv((this->d_color.getValue()*0.6).data());
//    //                glVertex3dv(points[2].data());glVertex3dv(points[3].data());glVertex3dv(points[7].data());glVertex3dv(points[6].data());

//    //                glColor3dv((this->d_color.getValue()*0.5).data());
//    //                glVertex3dv(points[0].data());glVertex3dv(points[1].data());glVertex3dv(points[5].data());glVertex3dv(points[4].data());

//    //                glColor3dv((this->d_color.getValue()*0.4).data());
//    //                glVertex3dv(points[3].data());glVertex3dv(points[1].data());glVertex3dv(points[5].data());glVertex3dv(points[7].data());

//    //                glColor3dv((this->d_color.getValue()*0.3).data());
//    //                glVertex3dv(points[2].data());glVertex3dv(points[0].data());glVertex3dv(points[4].data());glVertex3dv(points[6].data());
//    //            glEnd();
//    //        }

//        }
    }

    inline Index getKey(size_t i,size_t j,size_t k) const {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    inline type::Vec3i getCoord(const type::Vector3 & P) const {
        type::Vec3i cbox;
        for (int i = 0 ; i < 3 ; i++) {
            cbox[i] = floor(P[i]/m_cellSize[i]);
        }
//        cbox[0] = floor(P[0]/m_cellSize[0]);
//        cbox[1] = floor(P[1]/m_cellSize[1]);
//        cbox[2] = floor(P[2]/m_cellSize[2]);
        return cbox;
    }

    inline const type::Vector3 & getMin() const {
        return m_Bmin;
    }

    inline const type::Vector3 & getMax() const {
        return m_Bmax;
    }

    virtual type::Vec3i getBoxSize() const {
        return m_nbox;
    }

    virtual type::Vec3i getBoxCoord(const type::Vector3 & P) const {
        //compute the box where is P
        type::Vec3i cbox;
        for (int i = 0 ; i < 3 ; i++) {
            cbox[i] = floor((P[i] - m_Bmin[i])/m_cellSize[i]);
        }
//        cbox[0] = floor((P[0] - m_Bmin[0])/m_cellSize[0]);
//        cbox[1] = floor((P[1] - m_Bmin[1])/m_cellSize[1]);
//        cbox[2] = floor((P[2] - m_Bmin[2])/m_cellSize[2]);

        //project the box in the bounding box of the object
        //search with the closest box in bbox
        for (Index i=0;i<3;i++)
        {
            if (cbox[i] < 0) {
                cbox[i] = 0;
            } else if (cbox[i] >= m_nbox[i]) {
                cbox[i]=m_nbox[i]-1;
            }
        }

        return cbox;
    }

    virtual void recomputeNormals() {}

protected:
    bool m_staticInitDone;
    type::Vector3 m_Bmin,m_Bmax,m_cellSize;
    type::Vec3i m_nbox;
    type::Vec<2, size_t> m_offset;
//    std::vector<AABBBElement::SPtr> m_elements;
};


}
