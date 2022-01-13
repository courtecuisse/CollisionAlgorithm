#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/operations/Project.h>

namespace sofa::collisionAlgorithm {

/*!
 * \brief The AABBBroadPhaseGeometry class
 * Implementation of broad phase collision detection using bounding boxes
 */
class AABBBroadPhaseGeometry : public BaseGeometry {
    friend class AABBElement;

public:


    class AABBBElement : public TBaseElement<std::function<BaseProximity::SPtr(const AABBBElement *)> > {
    public:

        using Inherit = TBaseElement;
        typedef std::shared_ptr<AABBBElement> SPtr;



        void update() override {}

        inline BaseProximity::SPtr createProximity() const {
            return m_createProxFunc(this);
        }

        void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
            res.push_back(createProximity());
        }

        void insert(BaseElement::SPtr elmt) {
            m_elementId.insert(elmt);
        }

    private:
        std::set<BaseElement::SPtr> m_elementId;
    };

    SOFA_CLASS(AABBBroadPhaseGeometry,BaseGeometry);

    typedef BaseGeometry::BaseGeometry::Index Index;

    Data<type::Vec3i> d_nbox;
    Data<bool> d_refineBBox;
    Data<bool> d_static;

    core::objectmodel::SingleLink<AABBBroadPhaseGeometry, BaseGeometry, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    AABBBroadPhaseGeometry()
    : d_nbox(initData(&d_nbox, type::Vec3i(8,8,8),"nbox", "number of bbox"))
    , d_refineBBox(initData(&d_refineBBox, false,"refine", "Optimization to project center of box in order to find the minimal set of intersecting boxes"))
    , d_static(initData(&d_static, false,"isStatic", "Optimization: object is not moving in the scene"))
    , m_staticInitDone(false)
    {}

    virtual BaseElement::Iterator begin(Index eid = 0) const {

    }

    virtual size_t getOperationsHash() const {
        return typeid(AABBBroadPhaseGeometry).hash_code();
//        return l_geometry->getOperationsHash();
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

    /*!
     * \brief prepareDetection
     * checks if bounding boxes collided
     */
    void prepareDetection() {
        if (l_geometry == NULL) return;

        if(d_static.getValue() && m_staticInitDone)
            return;

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

        m_indexedElement.clear();
        m_offset[0] = m_nbox[1]*m_nbox[2];
        m_offset[1] = m_nbox[2];

        // center in -0.5 cellwidth
        m_Bmin -= m_cellSize * 0.5;
        m_Bmax -= m_cellSize * 0.5;

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

            const bool refine = d_refineBBox.getValue();

            auto projectOp = Operations::Project::func(l_geometry.get());

            for (int i=cminbox[0];i<cmaxbox[0];i++)
            {
                for (int j=cminbox[1];j<cmaxbox[1];j++)
                {
                    for (int k=cminbox[2];k<cmaxbox[2];k++)
                    {
                        if (refine) { // project the point on the element in order to know if the box is empty
                            type::Vector3 P = m_Bmin + m_cellSize*0.5;

                            P[0] += i*m_cellSize[0];
                            P[1] += j*m_cellSize[1];
                            P[2] += k*m_cellSize[2];

                            BaseProximity::SPtr prox = projectOp(P,elmt);
                            if (prox == NULL) continue;

                            type::Vector3 D = prox->getPosition();

                            if ((fabs(D[0])<=m_cellSize[0]*0.5) &&
                                (fabs(D[1])<=m_cellSize[1]*0.5) &&
                                (fabs(D[2])<=m_cellSize[2]*0.5))
                                getIndexedElement(i,j,k)->insert(elmt);
                        } else {
                           getIndexedElement(i,j,k)->insert(elmt);
                        }
                    }
                }
            }
        }
    }

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowBoundingCollisionModels()) return;

        if (this->l_geometry->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);

        glColor4fv(this->l_geometry->d_color.getValue().data());

        for (auto it = m_indexedElement.begin(); it != m_indexedElement.end(); it++) {
            unsigned eid = it->first;

            unsigned i = (eid) / m_offset[0];
            unsigned j = (eid - i*m_offset[0]) / m_offset[1];
            unsigned k = (eid - i*m_offset[0] - j*m_offset[1]);

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


    //        if (vparams->displayFlags().getShowWireFrame()) {
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
    //        } else {
    //            glBegin(GL_QUADS);
    //                glColor3dv((this->d_color.getValue()*0.8).data());
    //                glVertex3dv(points[0].data());glVertex3dv(points[1].data());glVertex3dv(points[3].data());glVertex3dv(points[2].data());

    //                glColor3dv((this->d_color.getValue()*0.7).data());
    //                glVertex3dv(points[4].data());glVertex3dv(points[5].data());glVertex3dv(points[7].data());glVertex3dv(points[6].data());

    //                glColor3dv((this->d_color.getValue()*0.6).data());
    //                glVertex3dv(points[2].data());glVertex3dv(points[3].data());glVertex3dv(points[7].data());glVertex3dv(points[6].data());

    //                glColor3dv((this->d_color.getValue()*0.5).data());
    //                glVertex3dv(points[0].data());glVertex3dv(points[1].data());glVertex3dv(points[5].data());glVertex3dv(points[4].data());

    //                glColor3dv((this->d_color.getValue()*0.4).data());
    //                glVertex3dv(points[3].data());glVertex3dv(points[1].data());glVertex3dv(points[5].data());glVertex3dv(points[7].data());

    //                glColor3dv((this->d_color.getValue()*0.3).data());
    //                glVertex3dv(points[2].data());glVertex3dv(points[0].data());glVertex3dv(points[4].data());glVertex3dv(points[6].data());
    //            glEnd();
    //        }

        }
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

    inline AABBBElement::SPtr getIndexedElement(size_t i,size_t j,size_t k) {
        Index key = getKey(i,j,k);

        auto it = m_indexedElement.find(key);
        if (it == m_indexedElement.end()) {
            AABBBElement::SPtr res = NULL;//AABBBElement::SPtr(new AABBBElement());
            m_indexedElement[key] = res;
            return res;
        } else {
            return it->second;
        }
    }

protected:
    type::Vector3 m_Bmin,m_Bmax,m_cellSize;
    type::Vec3i m_nbox;
    type::Vec<2, size_t> m_offset;
    std::map<Index, AABBBElement::SPtr > m_indexedElement;
    bool m_staticInitDone;
};


}
