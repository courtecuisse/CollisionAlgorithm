#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::collisionAlgorithm {

class PointElement;
class EdgeElement;
class TriangleElement;
class TetrahedronElement;

class PointElementSPtr;
class EdgeElementSPtr;
class TriangleElementSPtr;
class TetrahedronElementSPtr;

template<class ELMTSPtr>
class ElementContainer {
public:

    typedef typename std::vector<ELMTSPtr>::const_iterator const_iterator;

    void insert(const ELMTSPtr & e) {
        for (unsigned i=0;i<m_data.size();i++)
            if (e == m_data[i]) return;

        m_data.push_back(e);
    }

    ELMTSPtr operator[](unsigned i) { return m_data[i]; }

    const ELMTSPtr operator[](unsigned i) const { return m_data[i]; }

    void clear() { m_data.clear(); }

    unsigned size() const { return m_data.size(); }

    const_iterator cbegin() const { return m_data.cbegin(); }

    const_iterator cend() const { return m_data.cend(); }

private:
    std::vector<ELMTSPtr> m_data;
};

//: public std::enable_shared_from_this<BaseElement>
class BaseElement  {
public:

    typedef std::shared_ptr<BaseElement> SPtr;

    const ElementContainer<PointElementSPtr > & pointElements() const { return m_pointElements; }

    const ElementContainer<EdgeElementSPtr > & edgeElements() const { return m_edgeElements; }

    const ElementContainer<TriangleElementSPtr > & triangleElements() const { return m_triangleElements; }

    const ElementContainer<TetrahedronElementSPtr > & tetrahedronElements() const { return m_tetrahedronElements; }

    virtual void draw(const core::visual::VisualParams * vparams) = 0;

    virtual size_t getOperationsHash() const = 0;

    virtual std::string name() const = 0;

    virtual void update() = 0;

protected:

    ElementContainer<PointElementSPtr > & _pointElements() { return m_pointElements; }

    ElementContainer<EdgeElementSPtr > & _edgeElements() { return m_edgeElements; }

    ElementContainer<TriangleElementSPtr > & _triangleElements() { return m_triangleElements; }

    ElementContainer<TetrahedronElementSPtr > & _tetrahedronElements() { return m_tetrahedronElements; }

private:
    ElementContainer<PointElementSPtr > m_pointElements;
    ElementContainer<EdgeElementSPtr > m_edgeElements;
    ElementContainer<TriangleElementSPtr > m_triangleElements;
    ElementContainer<TetrahedronElementSPtr > m_tetrahedronElements;
};


}
