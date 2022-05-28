#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::collisionAlgorithm {

class PointElement;
class EdgeElement;
class TriangleElement;
class TetrahedronElement;

class BaseElement {
public:

    typedef std::shared_ptr<BaseElement> SPtr;

    const std::vector<std::shared_ptr<PointElement> > & pointElements() const { return m_pointElements; }

    const std::vector<std::shared_ptr<EdgeElement> > & edgeElements() const { return m_edgeElements; }

    const std::vector<std::shared_ptr<TriangleElement> > & triangleElements() const { return m_triangleElements; }

    const std::vector<std::shared_ptr<TetrahedronElement> > & tetrahedronElements() const { return m_tetrahedronElements; }

    virtual void draw(const core::visual::VisualParams * vparams) = 0;

    virtual size_t getOperationsHash() const = 0;

    virtual std::string name() const = 0;

    virtual void update() = 0;

protected:

    template<class ELMT>
    inline void insertElement(const ELMT * e) {
        std::vector<typename ELMT::SPtr> & v = elementVector<ELMT>();

        for (unsigned i=0;i<v.size();i++)
            if (e == v[i].get()) return;

        v.push_back(e->sptr());
    }

    template<class ELMT>
    inline void insertElement(typename ELMT::SPtr e) {
        std::vector<typename ELMT::SPtr> & v = elementVector<ELMT>();

        for (unsigned i=0;i<v.size();i++)
            if (e == v[i]) return;

        v.push_back(e);
    }

private:
    std::vector<std::shared_ptr<PointElement> > m_pointElements;
    std::vector<std::shared_ptr<EdgeElement> > m_edgeElements;
    std::vector<std::shared_ptr<TriangleElement> > m_triangleElements;
    std::vector<std::shared_ptr<TetrahedronElement> > m_tetrahedronElements;

    template<class ELMT>
    inline std::vector<std::shared_ptr<ELMT> > & elementVector();
};


template<>
inline std::vector<std::shared_ptr<PointElement> > & BaseElement::elementVector<PointElement>() { return m_pointElements; }

template<>
inline std::vector<std::shared_ptr<EdgeElement> > & BaseElement::elementVector<EdgeElement>() { return m_edgeElements; }

template<>
inline std::vector<std::shared_ptr<TriangleElement> > & BaseElement::elementVector<TriangleElement>() { return m_triangleElements; }

template<>
inline std::vector<std::shared_ptr<TetrahedronElement> > & BaseElement::elementVector<TetrahedronElement>() { return m_tetrahedronElements; }

}
