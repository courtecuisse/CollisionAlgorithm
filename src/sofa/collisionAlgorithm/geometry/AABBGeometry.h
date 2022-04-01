#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/operations/Project.h>

namespace sofa::collisionAlgorithm {

class AABBGeometry : public BaseGeometry {
public:

//    static inline long getKey(unsigned i, unsigned j, unsigned k) {
////        return i<<?? | j<<?? | k
//    }

//    static inline unsigned getIKey(long key) {

//    }

//    static inline unsigned getJKey(long key) {

//    }

//    static inline unsigned getKKey(long key) {

//    }

    class AABBBElement : public BaseElement {
    public:

        typedef std::shared_ptr<AABBBElement> SPtr;

        AABBBElement(const AABBGeometry * geo,unsigned key,unsigned i, unsigned j, unsigned k)
        : m_geometry(geo), m_key(key), m_i(i), m_j(j), m_k(k) {}

//        virtual unsigned id() override {
//            return m_key;
//        }

        size_t getOperationsHash() const override { return typeid(AABBBElement).hash_code(); }

        std::string name() const override { return "AABBBElement"; }

        void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
            res = m_projections;
        }

        void insert(BaseElement::SPtr elmt,BaseProximity::SPtr prox) {
            m_elements.push_back(elmt);
            m_projections.push_back(prox);
        }

        void draw(const core::visual::VisualParams * vparams) {
            unsigned i = m_i;//getIKey(m_key);
            unsigned j = m_j;//getJKey(m_key);
            unsigned k = m_k;//getKKey(m_key);

            type::Vector3 min = m_geometry->m_Bmin + type::Vector3((i  ) * m_geometry->m_cellSize[0],(j  ) * m_geometry->m_cellSize[1],(k  ) * m_geometry->m_cellSize[2]) ;
            type::Vector3 max = m_geometry->m_Bmin + type::Vector3((i+1) * m_geometry->m_cellSize[0],(j+1) * m_geometry->m_cellSize[1],(k+1) * m_geometry->m_cellSize[2]) ;
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

        const std::vector<BaseElement::SPtr> & elements() {
            return m_elements;
        }

    private:
        const AABBGeometry * m_geometry;
        unsigned m_key,m_i,m_j,m_k;
        std::vector<BaseElement::SPtr> m_elements;
        std::vector<BaseProximity::SPtr> m_projections;
    };

    class AABBBIterator : public ElementIterator {
    public:

        AABBBIterator(const AABBGeometry * geo)
        : m_geometry(geo) {
            m_iterator = geo->getElementsMap().cbegin();
        }

        bool end() const override {
            return m_iterator == m_geometry->getElementsMap().cend();
        }

        void next() override {
            m_iterator++;
        }

        std::shared_ptr<BaseElement> element() override {
            return m_iterator->second;
        }

        const std::shared_ptr<BaseElement> element() const override {
            return m_iterator->second;
        }

        size_t getOperationsHash() const override { return typeid(AABBBElement).hash_code(); }

    private:
        std::map<unsigned, AABBBElement::SPtr >::const_iterator m_iterator;
        const AABBGeometry * m_geometry;
    };

    SOFA_CLASS(AABBGeometry,BaseGeometry);

    Data<type::Vec3i> d_nbox;
    Data<bool> d_static;

    core::objectmodel::SingleLink<AABBGeometry, BaseGeometry, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;
    core::objectmodel::DataCallback c_nbox;

    AABBGeometry()
    : d_nbox(initData(&d_nbox, type::Vec3i(8,8,8),"nbox", "number of bbox"))
    , d_static(initData(&d_static, false,"isStatic", "Optimization: object is not moving in the scene"))
    , l_geometry(initLink("geometry", "link to geometry"))
    , m_staticInitDone(false) {
        c_nbox.addInputs({&d_nbox});
        c_nbox.addCallback(std::bind(&AABBGeometry::prepareDetection,this));
        l_geometry.setPath("@.");
    }

    ElementIterator::SPtr begin() const override {
        return ElementIterator::SPtr(new AABBBIterator(this));
    }

    virtual size_t getOperationsHash() const {
        return typeid(AABBGeometry).hash_code();
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const {
        return l_geometry->getState();
    }

    const std::map<unsigned, AABBBElement::SPtr > & getElementsMap() const {
        return m_indexedElement;
    }

    type::BoundingBox getBBox() const {
        return type::BoundingBox(m_Bmin,m_Bmax);
    }

    const std::vector<BaseElement::SPtr> & getElementSet(unsigned i, unsigned j, unsigned k) const {
        auto it = m_indexedElement.find(getKey(i,j,k));
        if (it == m_indexedElement.end()) {
            static std::vector<BaseElement::SPtr> empty;
            return empty;
        } else {
            const AABBBElement::SPtr box = it->second;
            return box->elements();
        }
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

        m_indexedElement.clear();

        m_offset[0] = m_nbox[1]*m_nbox[2];
        m_offset[1] = m_nbox[2];

        // center in -0.5 cellwidth
        m_Bmin -= m_cellSize * 0.5;
        m_Bmax -= m_cellSize * 0.5;

        auto projectOp = Operations::ProjectOperation::func(l_geometry->begin());

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
                            (fabs(D[2])<=m_cellSize[2]*0.5)) {

                            auto it = m_indexedElement.find(key);
                            AABBBElement::SPtr aabb_elmt = NULL;

                            //create the element in the map if it does not exists
                            if (it == m_indexedElement.cend()) {
                                aabb_elmt = AABBBElement::SPtr(new AABBBElement(this,key,i,j,k));
                                m_indexedElement[key] = aabb_elmt;
                            } else {
                                aabb_elmt = it->second;
                            }

                            aabb_elmt->insert(elmt,prox);
                        }
                    }
                }
            }
        }
    }

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowBoundingCollisionModels()) return;

        type::RGBAColor color = d_color.getValue();

        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        glColor4f(color[0],color[1],color[2],color[3]);
        for (auto it = begin();it != end(); it++) {
            it->element()->draw(vparams);
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

    //compute the box where is P
    inline type::Vec3i getBoxCoord(const type::Vector3 & P) const {
        return type::Vec3i((P[0] - m_Bmin[0])/m_cellSize[0],
                           (P[1] - m_Bmin[1])/m_cellSize[1],
                           (P[2] - m_Bmin[2])/m_cellSize[2]);
    }

    virtual void recomputeNormals() {}

protected:
    bool m_staticInitDone;
    type::Vector3 m_Bmin,m_Bmax,m_cellSize;
    type::Vec3i m_nbox;
    type::Vec<2, size_t> m_offset;
    std::map<unsigned, AABBBElement::SPtr > m_indexedElement;
};


}
