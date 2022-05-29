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

class BaseElement {
public:

    typedef std::shared_ptr<BaseElement> SPtr;

    const std::vector<PointElementSPtr > & pointElements() const { return m_pointElements; }

    const std::vector<EdgeElementSPtr > & edgeElements() const { return m_edgeElements; }

    const std::vector<TriangleElementSPtr > & triangleElements() const { return m_triangleElements; }

    const std::vector<TetrahedronElementSPtr > & tetrahedronElements() const { return m_tetrahedronElements; }

    virtual void draw(const core::visual::VisualParams * vparams) = 0;

    virtual size_t getOperationsHash() const = 0;

    virtual std::string name() const = 0;

    virtual void update() = 0;

protected:

    template<class ELMTSPtr>
    inline void insertElement(const ELMTSPtr & e) {
        std::vector<ELMTSPtr> & v = elementVector<ELMTSPtr>();

        for (unsigned i=0;i<v.size();i++)
            if (e == v[i]) return;

        v.push_back(e);
    }

private:
    std::vector<PointElementSPtr > m_pointElements;
    std::vector<EdgeElementSPtr > m_edgeElements;
    std::vector<TriangleElementSPtr > m_triangleElements;
    std::vector<TetrahedronElementSPtr > m_tetrahedronElements;

    template<class ELMTSPtr>
    inline std::vector<ELMTSPtr> & elementVector();
};


template<>
inline std::vector<PointElementSPtr > & BaseElement::elementVector<PointElementSPtr>() { return m_pointElements; }

template<>
inline std::vector<EdgeElementSPtr > & BaseElement::elementVector<EdgeElementSPtr>() { return m_edgeElements; }

template<>
inline std::vector<TriangleElementSPtr > & BaseElement::elementVector<TriangleElementSPtr>() { return m_triangleElements; }

template<>
inline std::vector<TetrahedronElementSPtr > & BaseElement::elementVector<TetrahedronElementSPtr>() { return m_tetrahedronElements; }

}
